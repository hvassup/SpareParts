#!/usr/bin/env python
# coding: utf-8

# In[1]:


import cv2
import numpy as np
import pandas as pd
from skimage.morphology import rectangle
from skimage.filters.rank import modal


# In[2]:


img = cv2.imread("1.jpg")


# In[3]:


# Color masking

mask = cv2.inRange(img, (200, 0, 0),(330, 360, 360))

cv2.imshow("Check it out!", mask)
cv2.waitKey()


# In[4]:


# Majority filtering

filtered_img = modal(mask,rectangle(10,10))

cv2.imshow("Check it out!", filtered_img)
cv2.waitKey()


# In[5]:


def find_boundaries(img, tresh):

    prev_col = 'n'
    cur_col = 'n'
    left_bounds = []
    right_bounds = []

    for i in range(img.shape[0]):
        if 255 in img[:,i]:
            cur_col = 'w'
        else:
            cur_col = 'b'
        if cur_col == 'w' and prev_col == 'b':
            left_bounds.append(i)
        elif cur_col == 'b' and prev_col == 'w':
            right_bounds.append(i)
        prev_col = cur_col
        
    for i in range(len(right_bounds)-1):
        if (left_bounds[i+1] - right_bounds[i]) < tresh:
            right_bounds[i] = 'x'
            left_bounds[i+1] = 'x'
    if 'x' in right_bounds:
        right_bounds.remove("x")
        left_bounds.remove("x")
    
    no_objects = len(left_bounds)
    
    return left_bounds, right_bounds, no_objects


# In[7]:


results = find_boundaries(filtered_img, 100)
print(f"Found {results[2]} objects in scene:")

for i in range(len(results[1])):
    print(f"{i+1}: {results[0][i]}, {results[1][i]}")


# In[ ]:




