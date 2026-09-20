#[allow(deprecated)] // mav gotta be mav
use mavlink::dialects::common::{MavMessage, SET_GPS_GLOBAL_ORIGIN_DATA};
use zenoh::Session;

use crate::{
    connection::{MavlinkHandle, SYSTEM_ID},
    packets::coords::Coords,
};

pub async fn declare_origin_sub(session: &Session, mav_handle: MavlinkHandle) {
    session
        .declare_subscriber("origin/set")
        .callback(move |sample| {
            let mav_handle = mav_handle.clone();
            tokio::spawn(async move {
                let bytes = &*sample.payload().to_bytes();
                if let Ok(coords) = rmp_serde::from_slice::<Coords>(bytes) {
                    let target_system = *SYSTEM_ID.get().expect("SYSTEM_ID not initialized");

                    println!(
                        "SETTING EKF ORIGIN: sysid={}, lat={}, lon={}, alt={}",
                        target_system, coords.lat_deg_e7, coords.lon_deg_e7, coords.alt_mm,
                    );

                    #[allow(deprecated)] // mav gotta be mav
                    let msg = MavMessage::SET_GPS_GLOBAL_ORIGIN(SET_GPS_GLOBAL_ORIGIN_DATA {
                        latitude: coords.lat_deg_e7,
                        longitude: coords.lon_deg_e7,
                        altitude: coords.alt_mm,
                        target_system: target_system,
                    });
                    mav_handle.send(msg).await.ok();
                    println!("ORIGIN SET: {coords:?}");
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize origin setter");
}
