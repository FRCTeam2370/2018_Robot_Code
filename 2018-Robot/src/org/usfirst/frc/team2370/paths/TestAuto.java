package org.usfirst.frc.team2370.paths;

import org.usfirst.frc.team2370.models.SrxMotionProfile;
import org.usfirst.frc.team2370.models.SrxTrajectory;

public class TestAuto extends SrxTrajectory{
	
	// WAYPOINTS:
	// (X,Y,degrees)
	// (0.00,0.00,0.00)
	// (3.00,3.00,0.00)
	
	public TestAuto() {
		this(false);
	}
			
    public TestAuto(boolean flipped) {
		super();
		
		double[][] leftPoints = {
				{0.005,0.105,10.000},
				{0.026,0.210,10.000},
				{0.073,0.471,10.000},
				{0.157,0.838,10.000},
				{0.288,1.309,10.000},
				{0.477,1.885,10.000},
				{0.733,2.564,10.000},
				{1.068,3.348,10.000},
				{1.491,4.234,10.000},
				{2.013,5.223,10.000},
				{2.645,6.314,10.000},
				{3.395,7.506,10.000},
				{4.275,8.797,10.000},
				{5.294,10.186,10.000},
				{6.461,11.672,10.000},
				{7.786,13.252,10.000},
				{9.279,14.925,10.000},
				{10.947,16.688,10.000},
				{12.801,18.539,10.000},
				{14.849,20.474,10.000},
				{17.098,22.491,10.000},
				{19.556,24.585,10.000},
				{22.232,26.753,10.000},
				{25.131,28.990,10.000},
				{28.260,31.292,10.000},
				{31.620,33.604,10.000},
				{35.208,35.874,10.000},
				{39.017,38.097,10.000},
				{43.044,40.270,10.000},
				{47.283,42.392,10.000},
				{51.729,44.458,10.000},
				{56.376,46.466,10.000},
				{61.217,48.412,10.000},
				{66.246,50.294,10.000},
				{71.457,52.107,10.000},
				{76.842,53.849,10.000},
				{82.394,55.515,10.000},
				{88.104,57.101,10.000},
				{93.964,58.604,10.000},
				{99.966,60.019,10.000},
				{106.096,61.298,10.000},
				{112.335,62.396,10.000},
				{118.667,63.313,10.000},
				{125.072,64.048,10.000},
				{131.532,64.602,10.000},
				{138.029,64.974,10.000},
				{144.546,65.164,10.000},
				{151.063,65.172,10.000},
				{157.562,64.997,10.000},
				{164.026,64.639,10.000},
				{170.436,64.098,10.000},
				{176.774,63.374,10.000},
				{183.020,62.468,10.000},
				{189.158,61.378,10.000},
				{195.169,60.106,10.000},
				{201.034,58.654,10.000},
				{206.736,57.021,10.000},
				{212.257,55.209,10.000},
				{217.579,53.222,10.000},
				{222.685,51.062,10.000},
				{227.559,48.733,10.000},
				{232.183,46.240,10.000},
				{236.542,43.589,10.000},
				{240.620,40.787,10.000},
				{244.405,37.844,10.000},
				{247.884,34.787,10.000},
				{251.047,31.635,10.000},
				{253.886,28.391,10.000},
				{256.392,25.055,10.000},
				{258.555,21.631,10.000},
				{260.367,18.124,10.000},
				{261.821,14.540,10.000},
				{262.910,10.887,10.000},
				{263.628,7.178,10.000},
				{263.970,3.424,10.000},
				{264.006,0.357,10.000},
				{264.420,4.148,10.000},
				{265.213,7.925,10.000},
				{266.380,11.665,10.000},
				{267.913,15.339,10.000},
				{269.805,18.917,10.000},
				{272.042,22.366,10.000},
				{274.607,25.650,10.000},
				{277.480,28.734,10.000},
				{280.638,31.580,10.000},
				{284.053,34.152,10.000},
				{287.695,36.415,10.000},
				{291.528,38.336,10.000},
				{295.517,39.887,10.000},
				{299.621,41.043,10.000},
				{303.800,41.786,10.000},
				{308.010,42.103,10.000},
				{312.209,41.989,10.000},
				{316.354,41.446,10.000},
				{320.402,40.484,10.000},
				{324.314,39.116,10.000},
				{328.050,37.367,10.000},
				{331.577,35.261,10.000},
				{334.859,32.830,10.000},
				{337.870,30.108,10.000},
				{340.584,27.132,10.000},
				{342.977,23.938,10.000},
				{345.034,20.564,10.000},
				{346.738,17.044,10.000},
				{348.080,13.414,10.000},
				{349.050,9.706,10.000},
				{349.645,5.948,10.000},
				{349.862,2.168,10.000},
				{350.023,1.611,10.000},
				{350.560,5.368,10.000},
				{351.468,9.086,10.000},
				{352.743,12.750,10.000},
				{354.378,16.347,10.000},
				{356.365,19.869,10.000},
				{358.696,23.306,10.000},
				{361.361,26.654,10.000},
				{364.352,29.907,10.000},
				{367.658,33.064,10.000},
				{371.270,36.123,10.000},
				{375.179,39.083,10.000},
				{379.373,41.945,10.000},
				{383.844,44.711,10.000},
				{388.582,47.382,10.000},
				{393.578,49.959,10.000},
				{398.823,52.447,10.000},
				{404.308,54.848,10.000},
				{410.024,57.164,10.000},
				{415.964,59.400,10.000},
				{422.120,61.559,10.000},
				{428.485,63.643,10.000},
				{435.050,65.657,10.000},
				{441.811,67.604,10.000},
				{448.759,69.487,10.000},
				{455.890,71.310,10.000},
				{463.198,73.075,10.000},
				{470.676,74.787,10.000},
				{478.321,76.447,10.000},
				{486.127,78.060,10.000},
				{494.090,79.628,10.000},
				{502.205,81.154,10.000},
				{510.469,82.640,10.000},
				{518.878,84.090,10.000},
				{527.429,85.506,10.000},
				{536.118,86.890,10.000},
				{544.943,88.245,10.000},
				{553.900,89.573,10.000},
				{562.988,90.876,10.000},
				{572.203,92.157,10.000},
				{581.545,93.418,10.000},
				{591.011,94.661,10.000},
				{600.600,95.888,10.000},
				{610.310,97.100,10.000},
				{620.140,98.300,10.000},
				{630.089,99.490,10.000},
				{640.156,100.672,10.000},
				{650.341,101.848,10.000},
				{660.643,103.018,10.000},
				{671.061,104.186,10.000},
				{681.597,105.353,10.000},
				{692.249,106.521,10.000},
				{703.018,107.692,10.000},
				{713.905,108.867,10.000},
				{724.910,110.049,10.000},
				{736.034,111.239,10.000},
				{747.278,112.440,10.000},
				{758.643,113.652,10.000},
				{770.131,114.879,10.000},
				{781.743,116.121,10.000},
				{793.481,117.382,10.000},
				{805.348,118.663,10.000},
				{817.344,119.967,10.000},
				{829.474,121.295,10.000},
				{841.739,122.650,10.000},
				{854.142,124.034,10.000},
				{866.687,125.450,10.000},
				{879.377,126.899,10.000},
				{892.215,128.386,10.000},
				{905.207,129.912,10.000},
				{918.355,131.480,10.000},
				{931.664,133.092,10.000},
				{945.139,134.753,10.000},
				{958.786,136.465,10.000},
				{972.609,138.230,10.000},
				{986.614,140.053,10.000},
				{1000.808,141.936,10.000},
				{1015.196,143.883,10.000},
				{1029.785,145.897,10.000},
				{1044.584,147.981,10.000},
				{1059.598,150.140,10.000},
				{1074.835,152.375,10.000},
				{1090.304,154.692,10.000},
				{1106.014,157.093,10.000},
				{1121.972,159.580,10.000},
				{1138.187,162.158,10.000},
				{1154.670,164.829,10.000},
				{1171.430,167.594,10.000},
				{1188.475,170.456,10.000},
				{1205.817,173.417,10.000},
				{1223.465,176.476,10.000},
				{1241.428,179.632,10.000},
				{1259.716,182.886,10.000},
				{1278.340,186.233,10.000},
				{1297.307,189.670,10.000},
				{1316.626,193.192,10.000},
				{1336.305,196.789,10.000},
				{1356.350,200.453,10.000},
				{1376.767,204.171,10.000},
				{1397.560,207.928,10.000},
				{1418.731,211.707,10.000},
				{1440.279,215.486,10.000},
				{1462.204,219.244,10.000},
				{1484.499,222.952,10.000},
				{1507.157,226.581,10.000},
				{1530.167,230.100,10.000},
				{1553.514,233.475,10.000},
				{1577.181,236.668,10.000},
				{1601.146,239.644,10.000},
				{1625.382,242.365,10.000},
				{1649.862,244.795,10.000},
				{1674.552,246.901,10.000},
				{1699.417,248.650,10.000},
				{1724.419,250.017,10.000},
				{1749.516,250.979,10.000},
				{1774.669,251.522,10.000},
				{1799.832,251.635,10.000},
				{1824.964,251.318,10.000},
				{1850.021,250.575,10.000},
				{1874.963,249.419,10.000},
				{1899.750,247.868,10.000},
				{1924.345,245.947,10.000},
				{1948.713,243.683,10.000},
				{1972.824,241.111,10.000},
				{1996.651,238.265,10.000},
				{2020.169,235.182,10.000},
				{2043.359,231.897,10.000},
				{2066.204,228.449,10.000},
				{2088.691,224.871,10.000},
				{2110.810,221.197,10.000},
				{2132.556,217.458,10.000},
				{2153.924,213.680,10.000},
				{2174.913,209.890,10.000},
				{2195.524,206.109,10.000},
				{2215.760,202.356,10.000},
				{2235.624,198.646,10.000},
				{2255.124,194.994,10.000},
				{2274.265,191.410,10.000},
				{2293.055,187.903,10.000},
				{2311.503,184.480,10.000},
				{2329.617,181.144,10.000},
				{2347.407,177.900,10.000},
				{2364.882,174.748,10.000},
				{2382.051,171.691,10.000},
				{2398.916,168.643,10.000},
				{2415.468,165.527,10.000},
				{2431.704,162.353,10.000},
				{2447.616,159.126,10.000},
				{2463.202,155.855,10.000},
				{2478.456,152.542,10.000},
				{2493.375,149.193,10.000},
				{2507.956,145.810,10.000},
				{2522.196,142.396,10.000},
				{2536.091,138.952,10.000},
				{2549.639,135.481,10.000},
				{2562.837,131.982,10.000},
				{2575.683,128.456,10.000},
				{2588.173,124.903,10.000},
				{2600.306,121.324,10.000},
				{2612.077,117.718,10.000},
				{2623.486,114.086,10.000},
				{2634.529,110.427,10.000},
				{2645.203,106.740,10.000},
				{2655.505,103.026,10.000},
				{2665.434,99.284,10.000},
				{2674.985,95.515,10.000},
				{2684.157,91.717,10.000},
				{2692.946,87.891,10.000},
				{2701.350,84.036,10.000},
				{2709.371,80.212,10.000},
				{2717.019,76.477,10.000},
				{2724.301,72.825,10.000},
				{2731.226,69.253,10.000},
				{2737.802,65.756,10.000},
				{2744.035,62.331,10.000},
				{2749.932,58.974,10.000},
				{2755.501,55.682,10.000},
				{2760.746,52.451,10.000},
				{2765.674,49.279,10.000},
				{2770.290,46.162,10.000},
				{2774.600,43.098,10.000},
				{2778.608,40.082,10.000},
				{2782.319,37.113,10.000},
				{2785.738,34.187,10.000},
				{2788.873,31.355,10.000},
				{2791.740,28.669,10.000},
				{2794.352,26.122,10.000},
				{2796.724,23.711,10.000},
				{2798.867,21.432,10.000},
				{2800.795,19.281,10.000},
				{2802.520,17.255,10.000},
				{2804.056,15.352,10.000},
				{2805.412,13.567,10.000},
				{2806.602,11.900,10.000},
				{2807.637,10.348,10.000},
				{2808.528,8.908,10.000},
				{2809.286,7.580,10.000},
				{2809.922,6.362,10.000},
				{2810.448,5.253,10.000},
				{2810.873,4.252,10.000},
				{2811.208,3.357,10.000},
				{2811.465,2.569,10.000},
				{2811.654,1.887,10.000},
				{2811.785,1.310,10.000},
				{2811.869,0.838,10.000},
				{2811.916,0.471,10.000},
				{2811.937,0.209,10.000},
				{2811.942,0.052,10.000},
				{2811.942,0.000,10.000}
		};
		
		double[][] rightPoints = {
				{0.005,0.105,10.000},
				{0.026,0.210,10.000},
				{0.073,0.472,10.000},
				{0.157,0.838,10.000},
				{0.288,1.310,10.000},
				{0.477,1.887,10.000},
				{0.734,2.569,10.000},
				{1.070,3.358,10.000},
				{1.495,4.252,10.000},
				{2.020,5.254,10.000},
				{2.656,6.363,10.000},
				{3.415,7.581,10.000},
				{4.306,8.909,10.000},
				{5.340,10.349,10.000},
				{6.531,11.901,10.000},
				{7.887,13.569,10.000},
				{9.423,15.353,10.000},
				{11.148,17.257,10.000},
				{13.077,19.283,10.000},
				{15.220,21.434,10.000},
				{17.591,23.713,10.000},
				{20.204,26.124,10.000},
				{23.071,28.670,10.000},
				{26.206,31.357,10.000},
				{29.625,34.189,10.000},
				{33.337,37.115,10.000},
				{37.345,40.084,10.000},
				{41.655,43.100,10.000},
				{46.272,46.164,10.000},
				{51.200,49.281,10.000},
				{56.445,52.454,10.000},
				{62.014,55.684,10.000},
				{67.911,58.976,10.000},
				{74.145,62.333,10.000},
				{80.720,65.759,10.000},
				{87.646,69.255,10.000},
				{94.929,72.828,10.000},
				{102.577,76.480,10.000},
				{110.598,80.215,10.000},
				{119.002,84.039,10.000},
				{127.791,87.894,10.000},
				{136.963,91.720,10.000},
				{146.515,95.518,10.000},
				{156.444,99.287,10.000},
				{166.747,103.029,10.000},
				{177.421,106.743,10.000},
				{188.464,110.429,10.000},
				{199.873,114.089,10.000},
				{211.645,117.721,10.000},
				{223.778,121.327,10.000},
				{236.268,124.906,10.000},
				{249.114,128.458,10.000},
				{262.312,131.984,10.000},
				{275.861,135.483,10.000},
				{289.756,138.955,10.000},
				{303.996,142.398,10.000},
				{318.577,145.812,10.000},
				{333.497,149.195,10.000},
				{348.751,152.544,10.000},
				{364.337,155.857,10.000},
				{380.250,159.129,10.000},
				{396.485,162.355,10.000},
				{413.038,165.530,10.000},
				{429.903,168.645,10.000},
				{447.072,171.693,10.000},
				{464.547,174.751,10.000},
				{482.337,177.902,10.000},
				{500.452,181.146,10.000},
				{518.900,184.482,10.000},
				{537.691,187.906,10.000},
				{556.832,191.413,10.000},
				{576.332,194.997,10.000},
				{596.197,198.649,10.000},
				{616.433,202.358,10.000},
				{637.044,206.112,10.000},
				{658.033,209.893,10.000},
				{679.401,213.683,10.000},
				{701.147,217.460,10.000},
				{723.267,221.200,10.000},
				{745.755,224.874,10.000},
				{768.600,228.451,10.000},
				{791.790,231.900,10.000},
				{815.308,235.184,10.000},
				{839.135,238.267,10.000},
				{863.246,241.113,10.000},
				{887.615,243.685,10.000},
				{912.210,245.948,10.000},
				{936.996,247.869,10.000},
				{961.938,249.420,10.000},
				{986.996,250.576,10.000},
				{1012.128,251.318,10.000},
				{1037.291,251.635,10.000},
				{1062.444,251.522,10.000},
				{1087.541,250.979,10.000},
				{1112.543,250.016,10.000},
				{1137.408,248.649,10.000},
				{1162.098,246.899,10.000},
				{1186.577,244.794,10.000},
				{1210.814,242.363,10.000},
				{1234.778,239.642,10.000},
				{1258.444,236.666,10.000},
				{1281.792,233.472,10.000},
				{1304.801,230.098,10.000},
				{1327.459,226.579,10.000},
				{1349.754,222.949,10.000},
				{1371.678,219.241,10.000},
				{1393.227,215.484,10.000},
				{1414.397,211.704,10.000},
				{1435.190,207.925,10.000},
				{1455.606,204.168,10.000},
				{1475.651,200.450,10.000},
				{1495.330,196.786,10.000},
				{1514.649,193.189,10.000},
				{1533.616,189.668,10.000},
				{1552.239,186.231,10.000},
				{1570.527,182.883,10.000},
				{1588.490,179.630,10.000},
				{1606.138,176.473,10.000},
				{1623.479,173.415,10.000},
				{1640.524,170.454,10.000},
				{1657.284,167.592,10.000},
				{1673.766,164.827,10.000},
				{1689.982,162.156,10.000},
				{1705.940,159.579,10.000},
				{1721.649,157.091,10.000},
				{1737.118,154.690,10.000},
				{1752.355,152.374,10.000},
				{1767.369,150.138,10.000},
				{1782.167,147.980,10.000},
				{1796.757,145.895,10.000},
				{1811.145,143.881,10.000},
				{1825.338,141.935,10.000},
				{1839.343,140.052,10.000},
				{1853.166,138.229,10.000},
				{1866.813,136.463,10.000},
				{1880.288,134.752,10.000},
				{1893.597,133.091,10.000},
				{1906.745,131.478,10.000},
				{1919.736,129.911,10.000},
				{1932.574,128.385,10.000},
				{1945.264,126.898,10.000},
				{1957.809,125.448,10.000},
				{1970.212,124.033,10.000},
				{1982.477,122.649,10.000},
				{1994.606,121.294,10.000},
				{2006.603,119.966,10.000},
				{2018.469,118.662,10.000},
				{2030.207,117.381,10.000},
				{2041.820,116.121,10.000},
				{2053.307,114.878,10.000},
				{2064.672,113.651,10.000},
				{2075.916,112.439,10.000},
				{2087.040,111.239,10.000},
				{2098.045,110.048,10.000},
				{2108.932,108.867,10.000},
				{2119.701,107.691,10.000},
				{2130.353,106.521,10.000},
				{2140.888,105.353,10.000},
				{2151.307,104.185,10.000},
				{2161.608,103.017,10.000},
				{2171.793,101.847,10.000},
				{2181.860,100.671,10.000},
				{2191.809,99.490,10.000},
				{2201.639,98.299,10.000},
				{2211.349,97.099,10.000},
				{2220.938,95.887,10.000},
				{2230.404,94.660,10.000},
				{2239.745,93.417,10.000},
				{2248.961,92.157,10.000},
				{2258.049,90.875,10.000},
				{2267.006,89.572,10.000},
				{2275.830,88.244,10.000},
				{2284.519,86.889,10.000},
				{2293.070,85.505,10.000},
				{2301.479,84.089,10.000},
				{2309.742,82.639,10.000},
				{2317.858,81.153,10.000},
				{2325.820,79.627,10.000},
				{2333.626,78.059,10.000},
				{2341.271,76.446,10.000},
				{2348.750,74.785,10.000},
				{2356.057,73.074,10.000},
				{2363.188,71.308,10.000},
				{2370.136,69.486,10.000},
				{2376.897,67.603,10.000},
				{2383.462,65.656,10.000},
				{2389.826,63.642,10.000},
				{2395.982,61.557,10.000},
				{2401.922,59.399,10.000},
				{2407.638,57.163,10.000},
				{2413.123,54.846,10.000},
				{2418.367,52.445,10.000},
				{2423.363,49.958,10.000},
				{2428.101,47.380,10.000},
				{2432.572,44.709,10.000},
				{2436.766,41.943,10.000},
				{2440.674,39.081,10.000},
				{2444.286,36.121,10.000},
				{2447.593,33.062,10.000},
				{2450.583,29.905,10.000},
				{2453.248,26.651,10.000},
				{2455.579,23.304,10.000},
				{2457.565,19.866,10.000},
				{2459.200,16.345,10.000},
				{2460.475,12.747,10.000},
				{2461.383,9.083,10.000},
				{2461.919,5.365,10.000},
				{2462.080,1.608,10.000},
				{2462.297,2.171,10.000},
				{2462.892,5.951,10.000},
				{2463.863,9.709,10.000},
				{2465.205,13.417,10.000},
				{2466.910,17.047,10.000},
				{2468.966,20.566,10.000},
				{2471.360,23.941,10.000},
				{2474.074,27.134,10.000},
				{2477.085,30.110,10.000},
				{2480.368,32.832,10.000},
				{2483.894,35.262,10.000},
				{2487.631,37.368,10.000},
				{2491.543,39.118,10.000},
				{2495.591,40.484,10.000},
				{2499.736,41.447,10.000},
				{2503.935,41.989,10.000},
				{2508.145,42.103,10.000},
				{2512.324,41.785,10.000},
				{2516.428,41.042,10.000},
				{2520.416,39.886,10.000},
				{2524.250,38.335,10.000},
				{2527.891,36.414,10.000},
				{2531.306,34.150,10.000},
				{2534.464,31.578,10.000},
				{2537.337,28.732,10.000},
				{2539.902,25.648,10.000},
				{2542.138,22.363,10.000},
				{2544.030,18.914,10.000},
				{2545.563,15.336,10.000},
				{2546.730,11.662,10.000},
				{2547.522,7.922,10.000},
				{2547.936,4.145,10.000},
				{2547.972,0.355,10.000},
				{2548.315,3.427,10.000},
				{2549.033,7.181,10.000},
				{2550.122,10.890,10.000},
				{2551.576,14.542,10.000},
				{2553.389,18.126,10.000},
				{2555.552,21.633,10.000},
				{2558.058,25.057,10.000},
				{2560.897,28.393,10.000},
				{2564.061,31.638,10.000},
				{2567.540,34.789,10.000},
				{2571.324,37.847,10.000},
				{2575.403,40.790,10.000},
				{2579.762,43.591,10.000},
				{2584.386,46.242,10.000},
				{2589.260,48.735,10.000},
				{2594.366,51.063,10.000},
				{2599.689,53.223,10.000},
				{2605.210,55.211,10.000},
				{2610.912,57.022,10.000},
				{2616.777,58.655,10.000},
				{2622.788,60.107,10.000},
				{2628.926,61.379,10.000},
				{2635.173,62.468,10.000},
				{2641.510,63.375,10.000},
				{2647.920,64.099,10.000},
				{2654.384,64.639,10.000},
				{2660.884,64.997,10.000},
				{2667.401,65.172,10.000},
				{2673.917,65.164,10.000},
				{2680.415,64.974,10.000},
				{2686.875,64.602,10.000},
				{2693.280,64.048,10.000},
				{2699.611,63.312,10.000},
				{2705.850,62.395,10.000},
				{2711.980,61.297,10.000},
				{2717.982,60.018,10.000},
				{2723.842,58.603,10.000},
				{2729.552,57.100,10.000},
				{2735.104,55.514,10.000},
				{2740.488,53.847,10.000},
				{2745.699,52.106,10.000},
				{2750.728,50.292,10.000},
				{2755.569,48.411,10.000},
				{2760.216,46.464,10.000},
				{2764.661,44.457,10.000},
				{2768.900,42.390,10.000},
				{2772.927,40.269,10.000},
				{2776.737,38.095,10.000},
				{2780.324,35.872,10.000},
				{2783.684,33.603,10.000},
				{2786.813,31.290,10.000},
				{2789.712,28.988,10.000},
				{2792.387,26.751,10.000},
				{2794.846,24.583,10.000},
				{2797.094,22.489,10.000},
				{2799.142,20.473,10.000},
				{2800.996,18.538,10.000},
				{2802.664,16.687,10.000},
				{2804.157,14.924,10.000},
				{2805.482,13.251,10.000},
				{2806.649,11.671,10.000},
				{2807.667,10.185,10.000},
				{2808.547,8.796,10.000},
				{2809.297,7.505,10.000},
				{2809.929,6.313,10.000},
				{2810.451,5.223,10.000},
				{2810.874,4.234,10.000},
				{2811.209,3.347,10.000},
				{2811.465,2.564,10.000},
				{2811.654,1.884,10.000},
				{2811.785,1.309,10.000},
				{2811.868,0.838,10.000},
				{2811.916,0.471,10.000},
				{2811.937,0.209,10.000},
				{2811.942,0.052,10.000},
				{2811.942,0.000,10.000}
		};
		
		if (flipped) {
			rightProfile = new SrxMotionProfile(leftPoints.length, leftPoints);
			leftProfile = new SrxMotionProfile(rightPoints.length, rightPoints);
		} else {
			leftProfile = new SrxMotionProfile(leftPoints.length, leftPoints);
			rightProfile = new SrxMotionProfile(rightPoints.length, rightPoints);
		}
	}

}