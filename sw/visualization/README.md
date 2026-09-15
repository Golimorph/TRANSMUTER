# Visualization

The visualizer renders a milling scene and replays its generated path over a configured duration.
During playback it cuts the posed tool geometry from the scene workpiece so the changing workpiece
is the dominant visual result.

Solid objects are accepted through `GeometryIf`. The renderer requests sampled visualization
vertices from that interface, so rendering does not depend on point-cloud storage. Ordered tool
poses are accepted through `ToolPathIf` for the same reason.

Material subtraction is delegated to `GeometryIf::cut`; representation-specific geometry logic
does not live in the visualizer.
