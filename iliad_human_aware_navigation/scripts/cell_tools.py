#!/usr/bin/env python

import math

def point2cell(point_x,point_y,x_min,x_max,y_min,y_max,grid_size):
	if point_x > x_max:
		print "Point x out of bounds"
		return -1,-1
	elif point_x == x_max:
		cell_x = math.floor((point_x - x_min)/grid_size) - 1
	else:
		cell_x = math.floor((point_x - x_min)/grid_size)
	
	if point_y > y_max:
		print "Point y out of bounds"
		return -1,-1
	elif point_y == y_max:
		cell_y = math.floor((point_y - y_min)/grid_size) - 1
	else:
		cell_y = math.floor((point_y - y_min)/grid_size)

	return int(cell_x),int(cell_y)


def cell2index(cell_x,cell_y,width,height):
	if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
		print "Cell out of bounds"
		return -1
	else:
		index = cell_x + cell_y*width 
		return int(index)

def point2index(point_x,point_y,x_min,x_max,y_min,y_max,grid_size,width,height):
	cell_x,cell_y = point2cell(point_x,point_y,x_min,x_max,y_min,y_max,grid_size)
	index = cell2index(cell_x,cell_y,width,height)
	return int(index)

def index2cell(index,width,height):
	if index < 0 or index >= width*height:
		print "Index out of bounds"
		return -1,-1
	else:
		cell_y = math.floor(index/width)
		cell_x = index - cell_y*width	
		return int(cell_x),int(cell_y)
