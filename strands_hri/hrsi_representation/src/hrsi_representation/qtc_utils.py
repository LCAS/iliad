# -*- coding: utf-8 -*-

import numpy as np

NO_STATE = 9.

def collapse_similar_states(qtc, qtc_type):
        """Collapses similar adjacent QTC states.

        :param qtc: a qtc state sequence

        :return: the input sequence without similar adjacent states
        """
        if len(qtc.shape) == 1:
            return qtc  # Only one state in chain.

        if qtc_type == "qtcbs":
            qtc = qtc[:,0:2].copy()

        # The nan handling is a bit hacky but fast and easy
        if qtc.dtype == np.float64: qtc[np.isnan(qtc)]=NO_STATE
        qtc = qtc[np.concatenate(([True],np.any(qtc[1:]!=qtc[:-1],axis=1)))]
        if qtc.dtype == np.float64: qtc[qtc==NO_STATE] = np.nan
        return qtc

def validate_qtc_sequence(qtc, qtc_type):
        """Removes illegal state transition by inserting necessary intermediate states

        :param qtc: a numpy array of the qtc state chain. One qtc state per row

        :return: The valid state chain as a numpy array
        """
        if len(qtc.shape) == 1:
            return np.array(qtc)  # Only one state in chain.

        if qtc_type == "qtcbs":
            qtc = qtc[:,0:2]

        legal_qtc = np.array([qtc[0,:]])

        for i in xrange(1, qtc.shape[0]):
            insert = np.array(qtc[i,:].copy())
            ###################################################################
            # self transition = 0, transition form - to + and vice versa = 2
            # transitions from - to 0 or + to 0 and vice versa = 1
            insert[np.abs(qtc[i-1,:]-insert)>1] = 0

            ###################################################################
            # find invalid transitions according to CND:
            # 1,2: -000 <> 0-00 | +000 <> 0+00 | -000 <> 0+00 | +000 <> 0-00
            # 1,3: -000 <> 00-0 | +000 <> 00+0 | -000 <> 00+0 | +000 <> 00-0
            # 1,4: -000 <> 000- | +000 <> 000+ | -000 <> 000+ | +000 <> 000-
            # 2,3: 0-00 <> 00-0 | 0+00 <> 00+0 | 0-00 <> 00+0 | 0+00 <> 00-0
            # 2,4: 0-00 <> 000- | 0+00 <> 000+ | 0-00 <> 000+ | 0+00 <> 000-
            # 3,4: 00-0 <> 000- | 00+0 <> 000+ | 00-0 <> 000+ | 00+0 <> 000-
            for j1 in xrange(0, len(qtc[i,:])-1):
                for j2 in xrange(j1+1, len(insert)):
                    if np.sum(np.abs(qtc[i-1,[j1,j2]])) == 1 \
                    and np.sum(np.abs(insert[[j1,j2]])) == 1:
                        if np.nanmax(np.abs(qtc[i-1,[j1,j2]] - insert[[j1,j2]])) > 0 \
                        and np.sum(qtc[i-1,[j1,j2]] - insert[[j1,j2]]) != 1:
                            insert[[j1,j2]] = 0

            #print insert

            if not np.array_equal(insert[np.logical_not(np.isnan(insert))], qtc[i, np.logical_not(np.isnan(insert))]):
                legal_qtc = np.append(legal_qtc, [insert], axis=0)

            legal_qtc = np.append(legal_qtc, [qtc[i,:]], axis=0)

        return legal_qtc