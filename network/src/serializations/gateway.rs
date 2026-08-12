use batman_robin::Gateway;
use serde::ser::SerializeStruct;
use serde::{Serialize, Serializer};

pub struct SerializableGateway<'a>(pub &'a Gateway);

impl Serialize for SerializableGateway<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let gateway = self.0;

        let mut state = serializer.serialize_struct("Gateway", 8)?;

        state.serialize_field("mac_addr", &gateway.mac_addr.to_string())?;
        state.serialize_field("router", &gateway.router.to_string())?;
        state.serialize_field("outgoing_if", &gateway.outgoing_if)?;

        state.serialize_field("downlink_kbps", &gateway.bandwidth_down)?;
        state.serialize_field("uplink_kbps", &gateway.bandwidth_up)?;
        state.serialize_field("throughput_kbps", &gateway.throughput)?;
        state.serialize_field("transmission_quality", &gateway.tq)?;

        state.serialize_field("is_best", &gateway.is_best)?;

        state.end()
    }
}
