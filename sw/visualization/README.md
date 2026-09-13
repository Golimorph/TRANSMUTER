# Visualization

The visualizer renders a milling scene and replays its generated path over a configured duration.
During playback it applies material removal to the scene workpiece so the changing workpiece is the
dominant visual result.

Solid objects are accepted through `GeometryIf`. The renderer requests sampled visualization
vertices from that interface, so rendering does not depend on point-cloud storage. The tool path
remains a `PointCloud` because it currently represents an ordered series of tool poses.

Material subtraction is delegated to `MaterialRemovalIf`; representation-specific geometry logic
does not live in the visualizer.
