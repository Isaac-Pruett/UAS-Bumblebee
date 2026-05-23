use std::time::Duration;
use zenoh;

mod name;
#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default()))
        .await
        .expect("failed to initialize zenoh");
    session
        .declare_queryable(name::topic("whoami"))
        .callback(|query| {
            tokio::spawn(async move {
                if let Some(n) = name::DRONE_NAME.get() {
                    if let Some(n) = rmp_serde::encode::to_vec(n).ok() {
                        query.reply(query.key_expr().clone(), n).await.ok();
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize whoami queryable");
    // Wait for subscribers to be ready
    tokio::time::sleep(Duration::from_millis(500)).await;

    // execute when cancelled
    tokio::signal::ctrl_c().await.ok();

    session.close().await.expect("failed to close session");
}
