# MuJoCo assets

Run `./setup_deps.sh assets` to download Go2/Go2W MJCF, meshes and textures from
[rl_sar_zoo](https://github.com/fan-ziqi/rl_sar_zoo). Generated directories are ignored by Git.
The downloader records the upstream revision and preserves available license notices.

Layout: `go2/mjcf/scene.xml`, `go2w/mjcf/scene.xml`, plus the resources each model references.
Use `-DASSET_DIR=/absolute/path/to/assets` to select another model directory.
