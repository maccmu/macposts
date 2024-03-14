"""Dynamic Traffic Assignment (DTA).

This module contains two classes for accessing the dynamic traffic assignment
functionalities in macposts. One is for single class DTA and the other is for
biclass DTA.

Note that this module is supposed to serve as an intermediate module between
the core `libmacposts' and the user.

"""

import numpy as np
import _macposts_ext as _ext


# XXX: I would like to use a common base class instead.
class _CommonMixin:
    """Mixin class for common methods of both Dta and Mcdta."""

    @classmethod
    def from_files(cls, directory):
        """Create an instance of *cls* with files in *directory*."""
        obj = cls()
        obj.initialize(str(directory))
        return obj

    def register_links(self, links=None):
        """Register *links* for recording cumulative curves.

        If a link is not registered, in order to save memory space, the
        cumulative curves for it will not be available after simulation.

        Note that *links* defaults to None, which means all links will be
        registered.

        """
        if links is None:
            links = self.links
        super().register_links(links)

    def _get_ccs(self, link_func, links):
        if links is None:
            links = self.registered_links
        ccs = np.empty((self.get_cur_loading_interval() + 1, len(links)))
        ccs[:] = np.nan
        for col, link in enumerate(links):
            cc = link_func(link)
            ticks = cc[:, 0].astype(int)
            ccs[ticks, col] = cc[:, 1]
        # Forward fill NaNs
        # Ref: https://stackoverflow.com/a/41191127
        mask = np.isnan(ccs)
        idxs = np.where(~mask, np.arange(mask.shape[0])[:, None], 0)
        np.maximum.accumulate(idxs, axis=0, out=idxs)
        ccs[mask] = ccs[idxs[mask], np.nonzero(mask)[1]]
        return ccs


class Dta(_CommonMixin, _ext.Dta):
    """Single class DTA."""

    def get_in_ccs(self, links=None):
        """Get the incoming cumulative curves for registered links.

        Required arguments *links* should be an iterable of link IDs and
        specify the desired links for which the cumulative curves will be
        retrieved. It could also be None, in which case all registered links
        will be used. For backward compatibility, if *links* is not iterable,
        it will be treated as a list of one element. However, that is not
        recommended.

        Return a Numpy array of shape (CURRENT-INTERVAL, NUM-LINKS).

        """
        return self._get_ccs(self.get_link_in_cc, links)

    def get_out_ccs(self, links=None):
        """Get the outgoing cumulative curves for registered links.

        Required arguments *links* should be an iterable of link IDs and
        specify the desired links for which the cumulative curves will be
        retrieved. It could also be None, in which case all registered links
        will be used. For backward compatibility, if *links* is not iterable,
        it will be treated as a list of one element. However, that is not
        recommended.

        Return a Numpy array of shape (CURRENT-INTERVAL, NUM-LINKS).

        """
        return self._get_ccs(self.get_link_out_cc, links)


class Mcdta(_CommonMixin, _ext.Mcdta):
    """Biclass DTA."""

    def get_car_in_ccs(self, links=None):
        """Get the incoming car cumulative curves for registered links.

        Required arguments *links* should be an iterable of link IDs and
        specify the desired links for which the cumulative curves will be
        retrieved. It could also be None, in which case all registered links
        will be used.

        Return a Numpy array of shape (CURRENT-INTERVAL, NUM-LINKS).

        """
        return self._get_ccs(self.get_car_link_in_cc, links)

    def get_car_out_ccs(self, links=None):
        """Get the outgoing car cumulative curves for registered links.

        Required arguments *links* should be an iterable of link IDs and
        specify the desired links for which the cumulative curves will be
        retrieved. It could also be None, in which case all registered links
        will be used.

        Return a Numpy array of shape (CURRENT-INTERVAL, NUM-LINKS).

        """
        return self._get_ccs(self.get_car_link_out_cc, links)

    def get_truck_in_ccs(self, links=None):
        """Get the incoming truck cumulative curves for registered links.

        Required arguments *links* should be an iterable of link IDs and
        specify the desired links for which the cumulative curves will be
        retrieved. It could also be None, in which case all registered links
        will be used.

        Return a Numpy array of shape (CURRENT-INTERVAL, NUM-LINKS).

        """
        return self._get_ccs(self.get_truck_link_in_cc, links)

    def get_truck_out_ccs(self, links=None):
        """Get the outgoing truck cumulative curves for registered links.

        Required arguments *links* should be an iterable of link IDs and
        specify the desired links for which the cumulative curves will be
        retrieved. It could also be None, in which case all registered links
        will be used.

        Return a Numpy array of shape (CURRENT-INTERVAL, NUM-LINKS).

        """
        return self._get_ccs(self.get_truck_link_out_cc, links)


# class Mmdta(_CommonMixin, _ext.Mmdta):
#     """Multi-modal DTA."""
