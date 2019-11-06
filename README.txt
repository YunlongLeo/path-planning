How to run our program:
	python fbsp path planning.py
Then input a block number (e.g. 10/20/40/60)


Evaluation:

10 obstacles(with max size 40cm):
Run 		RRT path length (cm) 	A* path length (cm) 	
1		448.89			138.31			
2		616.63			89.99			
3		1824.21			186.99			
4		4216.73			107.03			
5		112.07			59.21			
6		952.02			86.50		
7		73.09			127.62		
9		25.88			33.27		
9		737.04			126.49		
10		193.56			47.35			
Average 	920.01			91.28			
A*'s path is 10.0 times shorter than RRT.

20 obstacles(with max size 30cm):
Run 		RRT path length (cm) 	A* path length (cm) 	
1		1446.47			117.32			
2		550.19			95.60			
3		1000.70			176.70			
4		84.26			151.66			
5		112.07			59.21			
6		952.02			86.50		
7		73.09			127.62		
9		88.25			53.03		
9		236.07			134.95		
10		570.30			202.20			
Average 	511.34			120.47			
A*'s path is 4.25 times shorter than RRT.

40 obstacles(with max size 20cm):
Run 		RRT path length (cm) 	A* path length (cm) 	
1		127.13			87.23			
2		2460.77			104.80			
3		2418.50			89.35			
4		82.01			72.05			
5		174.46			63.47			
6		2188.43			130.01		
7		540.38			40.99		
9		2356.99			65.32		
9		90.89			118.36		
10		116.92			138.90			
Average 	1055.65			91.05			
A*'s path is 11.6 times shorter than RRT.


60 obstacles(with max size 10cm):
Run 		RRT path length (cm) 	A* path length (cm) 	
1		65.73			59.46			
2		150.32			112.35			
3		2793.09			44.28			
4		85.27			75.41			
5		4818.16			84.65			
6		2977.67			144.23		
7		2524.54			99.28		
9		509.99			51.19		
9		505.74			66.46		
10		28.51			61.71			
Average 	1442.02			89.03			
A*'s path is 16.2 times shorter than RRT.

In general, in terms of path, A* algorithm is 10 times shorter than RRT algorithm. However in some cases, RRT has shorter path. The reason is that there is no obstacle between starting point and goal point. Thus the RRT algorithm can generate an almost straight line between starting point and goal point. In terms of time, RRT has very unstable running time. Sometimes it is even faster than A* because the specific map does not have obstacle on its way, sometimes it is very slow and has very long path length because the map has big obstacle on its way. Even with path length around 1000cm, RRT still has almost the same running time comparing to A* because A* needs more time at each stage. Overall, A* algorithm has shorter and more stable path length and running time.


