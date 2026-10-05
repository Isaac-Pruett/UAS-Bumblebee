import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import zenoh
from foxglove.messages import Color, SceneEntity, SceneUpdate, SpherePrimitive, Vector3

from telemetry.main import main


class FormationCenterTests(unittest.TestCase):
    def test_published_center_reaches_blue_enu_scene(self):
        # glam::Vec3 encoded by rmp_serde; mirrored by the Rust wire-format test.
        payload = bytes.fromhex("93 ca3f800000 ca40000000 ca40a00000")
        sample = SimpleNamespace(
            payload=zenoh.ZBytes(payload),
            key_expr=zenoh.KeyExpr("formations/target"),
            kind=zenoh.SampleKind.PUT,
            timestamp=None,
        )
        session = Mock()
        callbacks = {}

        def subscribe(topic, handler):
            callbacks[topic] = handler
            return Mock()

        session.declare_subscriber.side_effect = subscribe

        def receive_then_stop(_):
            callbacks["formations/target"](sample)
            callbacks["**"](sample)
            raise KeyboardInterrupt

        with (
            patch("telemetry.main.zenoh.open", return_value=session),
            patch("telemetry.main.foxglove.start_server") as server,
            patch("telemetry.main.foxglove.log") as log,
            patch("telemetry.main.SceneEntity", wraps=SceneEntity) as entity,
            patch("telemetry.main.SpherePrimitive", wraps=SpherePrimitive) as sphere,
            patch("telemetry.main.Vector3", wraps=Vector3) as vector,
            patch("telemetry.main.Color", wraps=Color) as color,
            patch("telemetry.main.time.sleep", side_effect=receive_then_stop),
            patch.dict("os.environ", {}, clear=True),
        ):
            main()

        scenes = [
            call.args
            for call in log.call_args_list
            if call.args[0] == "formations/target"
        ]
        self.assertEqual(len(scenes), 1)
        update = scenes[0][1]
        self.assertIsInstance(update, SceneUpdate)
        # SDK messages are write-only; inspect inputs while constructing real messages.
        entity.assert_called_once()
        self.assertEqual(entity.call_args.kwargs["id"], "formation_center")
        self.assertEqual(entity.call_args.kwargs["frame_id"], "enu")
        sphere.assert_called_once()
        vector.assert_any_call(x=1.0, y=2.0, z=5.0)
        vector.assert_any_call(x=0.25, y=0.25, z=0.25)
        color.assert_called_once_with(r=0.0, g=0.0, b=1.0, a=0.8)
        self.assertTrue(update.encode())
        self.assertTrue(
            any(call.args[0] == "/raw/formations/target" for call in log.call_args_list)
        )
        session.close.assert_called_once()
        server.return_value.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
