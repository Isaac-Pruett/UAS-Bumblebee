use crate::get_session;
#[allow(deprecated)] // mav gotta be mav
use mavlink::dialects::common::{MavMessage, SET_GPS_GLOBAL_ORIGIN_DATA};

use crate::{
    connection::{MavlinkHandle, SYSTEM_ID},
    packets::coords::Coords,
};

/// Replies with an empty payload when queued for MAVLink, or an error on failure.
/// Success does not confirm that the flight controller applied the origin.
pub async fn declare_origin_queryable(mav_handle: MavlinkHandle) {
    get_session()
        .declare_queryable("origin/set")
        .callback(move |query| {
            let mav_handle = mav_handle.clone();
            tokio::spawn(async move {
                let coords = query
                    .payload()
                    .and_then(|payload| rmp_serde::from_slice::<Coords>(&payload.to_bytes()).ok());
                let mut error = match (&coords, SYSTEM_ID.get()) {
                    (None, _) => Some("missing or invalid origin coordinates"),
                    (_, None) => Some("SYSTEM_ID not initialized"),
                    _ => None,
                };

                if let (Some(coords), Some(&target_system)) = (coords, SYSTEM_ID.get()) {
                    println!(
                        "SETTING EKF ORIGIN: sysid={}, lat={}, lon={}, alt={}",
                        target_system, coords.lat_deg_e7, coords.lon_deg_e7, coords.alt_mm,
                    );

                    #[allow(deprecated)] // mav gotta be mav
                    let msg = MavMessage::SET_GPS_GLOBAL_ORIGIN(SET_GPS_GLOBAL_ORIGIN_DATA {
                        latitude: coords.lat_deg_e7,
                        longitude: coords.lon_deg_e7,
                        altitude: coords.alt_mm,
                        target_system,
                    });
                    if mav_handle.send(msg).await.is_ok() {
                        println!("ORIGIN SET QUEUED: {coords:?}");
                    } else {
                        error = Some("failed to queue origin set for MAVLink");
                    }
                }

                let reply = match error {
                    Some(error) => query.reply_err(error).await,
                    None => query.reply("origin/set", "").await,
                };
                if let Err(err) = reply {
                    eprintln!("failed to reply to origin query: {err}");
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize origin queryable");
}
