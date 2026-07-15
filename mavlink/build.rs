fn main() {
    println!("cargo:rerun-if-env-changed=PWM_ACTUATORS_PER_PAGE");

    let per_page = std::env::var("PWM_ACTUATORS_PER_PAGE").unwrap_or_else(|_| "6".to_string());

    println!("cargo:rustc-env=PWM_ACTUATORS_PER_PAGE={per_page}");
}
