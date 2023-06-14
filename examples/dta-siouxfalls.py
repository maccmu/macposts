"""Single class DTA with adaptive routing on the Sioux Falls network[0].

This script downloads data from our server[1] and run single class DTA with the
downloaded data. If you trust us (hint: you should not), you may directly run
this script. Otherwise, please download the data manually from [1] and verify
the files. Then put the unzipped directory under the "data" directory and run
this script.

[0]: https://github.com/bstabler/TransportationNetworks/tree/master/SiouxFalls
[1]: https://babyam.andrew.cmu.edu/files/macposts/data/siouxfalls.zip

"""

import macposts
import urllib.request as r
import sys
import io
import hashlib
import zipfile
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

DATA_DIRECTORY = Path("data")
DATA_URL = "https://babyam.andrew.cmu.edu/files/macposts/data/siouxfalls.zip"
B2SUM = "885cd1b256af15f5f733eba5c9ae33c38861361a09fe9c6d30655da83e0a0f0dc9aecb13c85d9238df8affdf42a231e23fac78eddd2349cab6dfce2030dd5711"
prng = np.random.default_rng(123)


def ensure_data(directory):
    directory = Path(directory)
    if (directory / "siouxfalls").exists():
        return
    directory.mkdir(parents=True, exist_ok=True)
    with r.urlopen(DATA_URL) as f:
        contents = f.read()
        b2 = hashlib.blake2b(contents, digest_size=64).hexdigest()
        if b2 != B2SUM:
            raise RuntimeError("Failed to verify the integrity of data file")
        with io.BytesIO(contents) as bf, zipfile.ZipFile(bf) as zf:
            zf.extractall(DATA_DIRECTORY)


def run():
    ensure_data(DATA_DIRECTORY)
    dta = macposts.Dta.from_files(DATA_DIRECTORY / "siouxfalls")
    dta.register_links()
    dta.install_cc()
    dta.run_whole()
    return dta


if __name__ == "__main__":
    try:
        dta = run()
        in_ccs = dta.get_in_ccs()
        out_ccs = dta.get_out_ccs()

        # Draw cumulative curves for four links
        nlinks = 8 # Number of links to plot
        lidxs = prng.choice(np.arange(len(dta.links)), size=nlinks,
                            replace=False)
        lidxs.sort()
        fig, axs = plt.subplots(nrows=2, ncols=(nlinks + 1) // 2,
                                sharex=True, sharey=True, figsize=(10, 4))
        for lidx, ax in zip(lidxs, axs.flat):
            in_cc = in_ccs[:, lidx]
            out_cc = out_ccs[:, lidx]
            ax.plot(np.arange(in_cc.size), in_cc, label="in")
            ax.plot(np.arange(out_cc.size), out_cc, label="out")
            ax.set_title("link {}".format(dta.links[lidx]))
        fig.legend(*ax.get_legend_handles_labels(),
                   loc="lower right", ncols=2, bbox_to_anchor=(0.95, 0))
        fig.supxlabel("interval")
        fig.supylabel("number of vehicles")
        fig.tight_layout()
        plt.show()

    except KeyboardInterrupt:
        print("Interrupted by user; stopping now…", file=sys.stderr)
        pass
