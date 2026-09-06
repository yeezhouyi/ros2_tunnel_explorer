"""map_loader tests: PGM/YAML pair -> occupancy dict, polarity guard."""
import numpy as np

from cleaning_mode.map_loader import load_map_grid, read_pgm


def write_pgm(path, img):
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.astype(np.uint8).tobytes())


def write_yaml(path, image="m.pgm", res=0.05, origin=(0.0, 0.0)):
    path.write_text(
        "image: %s\nresolution: %s\norigin: [%s, %s, 0.0]\n"
        % (image, res, origin[0], origin[1]))


def test_load_standard_polarity(tmp_path):
    img = np.full((10, 10), 254, dtype=np.uint8)
    img[:, 0] = 0  # black wall column
    write_pgm(tmp_path / "m.pgm", img)
    write_yaml(tmp_path / "m.yaml", origin=(1.0, 2.0))
    g = load_map_grid(str(tmp_path / "m.yaml"))
    assert g["obstacle"][:, 0].all()
    assert g["known_free"][:, 1:].all()
    assert g["meta"]["resolution"] == 0.05
    assert g["meta"]["origin"] == [1.0, 2.0]


def test_load_inverted_polarity_guard(tmp_path):
    img = np.zeros((8, 8), dtype=np.uint8)  # mostly black
    write_pgm(tmp_path / "m.pgm", img)
    write_yaml(tmp_path / "m.yaml", res=0.1)
    g = load_map_grid(str(tmp_path / "m.yaml"))
    assert g["known_free"].all()  # polarity guard flipped


def test_unknown_band(tmp_path):
    img = np.full((6, 6), 254, dtype=np.uint8)
    img[0:2, :] = 205  # grey unknown band
    write_pgm(tmp_path / "m.pgm", img)
    write_yaml(tmp_path / "m.yaml")
    g = load_map_grid(str(tmp_path / "m.yaml"))
    assert g["unknown"][0:2, :].all()
    assert g["known_free"][2:, :].all()
