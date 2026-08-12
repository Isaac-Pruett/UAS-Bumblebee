use batman_robin::Neighbor;

pub struct SerializableNeighbor<'a>(&'a Neighbor);

impl Serialize for SerializableNeighbor<'_> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let neighbor = self.0;

        let mut state = serializer.serialize_struct("Neighbor", 3)?;
        state.serialize_field("last_seen_ms", &neighbor.last_seen_ms)?;
        state.serialize_field("outgoing_if", &neighbor.outgoing_if)?;
        state.serialize_field("kbps", &neighbor.throughput_kbps.unwrap_or(0))?;
        state.end()
    }
}
