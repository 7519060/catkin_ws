#! /usr/bin/env python
#coding: UTF-8

import numpy as np
import pandas as pd

###orig
# csvdata = pd.read_csv('./pinv_int_mat/pinv_int_mat.csv', header=None)
# csvdata.to_pickle('./pinv_int_mat_pickle/pinv_int_mat.pickle')

# pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat.pickle')
# pinv_int_mat = np.array
# pinv_int_mat = pickledata.values

###double
print('henkan start')
csvdata = pd.read_csv('./pinv_int_mat/pinv_int_mat_double.csv', header=None)
print('read csv file')
csvdata.to_pickle('./pinv_int_mat_pickle/pinv_int_mat_double.pickle')
print('done')

###normalized
# csvdata = pd.read_csv('./pinv_int_mat/pinv_int_mat_normalized.csv', header=None)
# csvdata.to_pickle('./pinv_int_mat_pickle/pinv_int_mat_normalized.pickle')