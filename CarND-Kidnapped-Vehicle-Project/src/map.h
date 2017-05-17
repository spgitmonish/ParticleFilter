/*
 * map.h
 *
 *  Created on: Dec 12, 2016
 *      Author: mufferm
 */

#ifndef MAP_H_
#define MAP_H_

class Map {
public:

	struct single_landmark_s{
		// Landmark ID
		int id_i ;
		// Landmark x-position in the map (global coordinates)
		float x_f;
		// Landmark y-position in the map (global coordinates)
		float y_f;
	};

  // List of landmarks in the map
	std::vector<single_landmark_s> landmark_list ;
};



#endif /* MAP_H_ */
