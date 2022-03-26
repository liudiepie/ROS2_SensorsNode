#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   ransac.py
@Time    :   2022/03/26 01:00:34
@Author  :   Cheng Liu 
@Version :   1.0
@Contact :   cliu@umd.edu
@License :   (C)Copyright 2022-2023, Cheng Liu
@Desc    :   None
'''

import numpy as np
import math

#ransac method
def ransac(x, y):
    A = np.stack((x, np.ones((len(x)), dtype = int)), axis=1)
    #sett a threshold value
    threshold = np.std(y)/3
    ransac_pred = ransac_fit(A, y, 3, threshold)
    ransac_y = A.dot(ransac_pred)
    return ransac_y

def ransac_fit(A, y, sample, threshold):
    #initialize
    num_iter = math.inf
    iter_comp = 0
    max_inlier = 0
    best_fit = None
    prob_outlier = 0
    prob_desired = 0.70 #desired probability
    combined_data = np.column_stack((A,y))
    data_len = len(combined_data)

    #find the number of iterations
    while num_iter > iter_comp:
        #shuffling the rows and taking the first rows
        np.random.shuffle(combined_data)
        sample_data = combined_data[:sample, :]
       
        #estimating the y 
        pred_model = ls_fit(sample_data[:,:-1], sample_data[:, -1:])
      
        #count the inliers 
        y_inliers = A.dot(pred_model)
        err = np.abs(y - y_inliers.T)
       
        #if err is less than the threshold value, then the point is an inlier
        inlier_count = np.count_nonzero(err<threshold)
        #print('Inlier Count', inlier_count)
        
        #best fit with maximum inlier count
        if inlier_count > max_inlier:
            max_inlier = inlier_count
            best_fit = pred_model

        #calculating the outlier 
        prob_outlier = 1 - inlier_count/data_len

        #computing the number of iterations 
        num_iter = math.log(1 - prob_desired)/math.log(1-(1-prob_outlier)**sample)
        iter_comp = iter_comp + 1
    return best_fit

def ls_fit(A, y):
    A_trans = A.transpose()
    ATA = A_trans.dot(A)
    ATY = A_trans.dot(y)
    pred = (np.linalg.inv(ATA)).dot(ATY)
    return pred
