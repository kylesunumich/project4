// Project Identifier: 1761414855B69983BD8035097EFBD312EB0527F0
//
//  drone.cpp
//  project4
//
//  Created by Kyle Sun on 4/5/20.
//  Copyright © 2020 Kyle Sun. All rights reserved.
//

#include "xcode_redirect.hpp"
#include <getopt.h>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>


// ----------------------------------------------------------------------------
//                    LocationType Declarations (Enumerated Class)
// ----------------------------------------------------------------------------

enum class LocationType { Medical, Border, Normal, Empty};

// ----------------------------------------------------------------------------
//                    Location Declarations
// ----------------------------------------------------------------------------

class Location {
    
public:
    
    Location();
    
    Location(int x_coord_in, int y_coord_in, int location_num_in, char mode_in);
    
    int get_x_coord();
    int get_y_coord();
    LocationType get_location_type();
    int get_location_num();
    
private:
    
    int x_coord;
    int y_coord;
    
    // Location 0, 1, 2, 3, ect...
    int location_num;
    
    // Medical: if x and y are both negative
    // ex: (x = (-), y = (-))
    // Border: if one is negative and other is 0, OR (0,0)
    // ex: (x = 0, y = (-)) OR (y = 0, x = (-)) OR (x = 0, y = 0)
    
    LocationType location_type;
    
};

// ----------------------------------------------------------------------------
//                    Drone Declarations
// ----------------------------------------------------------------------------

class Drone {
    
public:
    
//    struct Node {
//
//        int location_index;
//        Node* prev;
//        Node* next;
//
//    };
    
    Drone();
    
    void get_options(int argc, char** argv);
    
    char get_mode();
    
    double get_distance(Location &l1, Location &l2);
    
    void read_input();
    
    // PART A: MST //
    
    void run_MST();
    
    void prim_algorithm();
    
    void prim_initialize_vectors(Location &first_location, size_t first_location_index);
    
    void prim_algorithm_update(Location &next_location, size_t next_location_index);
    
    size_t find_closest_location();
    
    double MST_get_total_distance();
    
    void MST_print();
    
    // PART B: FASTTSP //
    
    void run_FASTTSP();
    
    void FAST_initialize_vectors(size_t first_index, size_t second_index, size_t third_index, double &total_distance);
    
    void FAST_initialize_distance_vector();
    
    void FAST_arbitrary_insert_algorithm(double &total_distance);
    
    void FAST_print(double total_distance);
    
    double FAST_distance_change(size_t first_index, size_t second_index, size_t new_index);
    
    // PART C: OPTTSP //
    
    void run_OPTTSP();
    
    void genPerms(size_t permLength);
    
    bool is_promising(size_t permLength);
    
    void OPT_initialize();
    
    void OPT_FASTTSP_helper();
    
    void OPT_modified_prim_update(Location &next_location, size_t next_location_index, std::vector<size_t> &unvisited_locations);
    
    void OPT_modified_prim_initialize_vectors(Location &first_location, size_t first_location_index, std::vector<size_t> &unvisited_locations);
    
    void OPT_modified_prim_algorithm(std::vector<size_t> &unvisited_locations);
    
    void OPT_reset_prim();
    
    void OPT_print();
    
private:
    
    // 'N' by default; must be 'M' for MST, 'F' for FASTTSP, or 'O' for OPTTSP
    char mode;
    
    int num_locations;
    
    // For all vectors:
    // Index of locations corresponds to location num
    // ex: Index 0 stores Location 0
    std::vector<Location> v_locations;
    
    // ----------------------------------------------------------------------------
    //                    PART A
    // ----------------------------------------------------------------------------
    
    // Parent location
    std::vector<Location> prim_parents;
    
    // Distance from parent
    std::vector<double> prim_distances;
    
    // if visited or not
    std::vector<bool> prim_visited;
    
    // ----------------------------------------------------------------------------
    //                    PART B
    // ----------------------------------------------------------------------------
    
    std::vector<size_t> FAST_path;
    
    
    // ----------------------------------------------------------------------------
    //                    PART C
    // ----------------------------------------------------------------------------
    
    
    std::vector<size_t> OPT_path;
    
    std::vector<size_t> OPT_best_path;
    
    double OPT_best_distance;
    
    double OPT_current_distance;
    
    //std::vector<bool> OPT_visited;
    
};


//// ----------------------------------------------------------------------------
////                    Optimal Declarations
//// ----------------------------------------------------------------------------
//
//class Optimal {
//
//public:
//
//    template <typename T>
//    void genPerms(size_t permLength) {
//        if (permLength == path.size()) {
//
//            // add closing edge
//            // check/update best distance/path
//            // subtract closing edge
//
//
//            return;
//        } // if
//        if (!promising(path, permLength))
//            return;
//        for (size_t i = permLength; i < path.size(); ++i) {
//            swap(path[permLength], path[i]);
//            genPerms(path, permLength + 1);
//            swap(path[permLength], path[i]);
//        } // for
//    } // genPerms()
//
//
//private:
//
//    std::vector<size_t> &OPT_path;
//
//    std::vector<size_t> &OPT_best_path;
//
//    double OPT_running_total;
//
//    double OPT_best_distance;
//
//
//};



// ----------------------------------------------------------------------------
//                               Driver
// ----------------------------------------------------------------------------

int main(int argc, char** argv) {
    
    xcode_redirect(argc, argv);
    std::ios_base::sync_with_stdio(false);
    
    std::cout << std::setprecision(2); // Always show 2 decimal places
    std::cout << std::fixed; // Disable scientific notation for large numbers
    
    Drone d1;
    
    d1.get_options(argc, argv);
    
    // MST mode
    if (d1.get_mode() == 'M') {
        
        d1.run_MST();
        
    }
    
    else if (d1.get_mode() == 'F') {
        
        d1.run_FASTTSP();
        
    }
    
    else if (d1.get_mode() == 'O') {
        
        
        d1.run_OPTTSP();
        
    }
    
    else {
        
        std::cerr << "Error: Invalid mode: '" << d1.get_mode() << "' read in from getOpt. Program terminating\n";
        
        exit(1);
        
    }

    return 0;
    
}


// ----------------------------------------------------------------------------
//                    Location Definitions
// ----------------------------------------------------------------------------

Location::Location() {
    
    // Default values; only appear if uninitialized
    location_type = LocationType::Empty;
    
    location_num = -1;
    
}

Location::Location(int x_coord_in, int y_coord_in, int location_num_in, char mode_in) {
    
    location_num = location_num_in;
    
    x_coord = x_coord_in;
    y_coord = y_coord_in;
    
    // Medical: if x and y are both negative
    // ex: (x = (-), y = (-))
    // Border: if one is negative and other is 0, OR (0,0)
    // ex: (x = 0, y = (-)) OR (y = 0, x = (-)) OR (x = 0, y = 0)
    
    if (mode_in == 'M') { // need LocationType
        
        if (x_coord < 0 && y_coord < 0) {
            
            location_type = LocationType::Medical;
            
        }
        
        else if ((x_coord < 0 && y_coord == 0) ||
                 (y_coord < 0 && x_coord == 0) ||
                 (x_coord == 0 && y_coord == 0)) {
            
            location_type = LocationType::Border;
            
        }
        
        else {
            
            location_type = LocationType::Normal;
            
        }
        
    }
    
}

int Location::get_x_coord() {
    
    return x_coord;
    
}

int Location::get_y_coord() {
    
    return y_coord;
    
}

LocationType Location::get_location_type() {
    
    return location_type;
    
}

int Location::get_location_num() {
    
    return location_num;
    
}

// ----------------------------------------------------------------------------
//                    Drone Definitions
// ----------------------------------------------------------------------------

// Default constructor
Drone::Drone() {
    
    mode = 'N';
    
}

void Drone::get_options(int argc, char** argv) {
    
    int option_index = 0, option = 0;
    
    // Don't display getopt error messages about options
    opterr = false;
    
    // use getopt to find command line options
    struct option longOpts[] = {{ "mode", required_argument, nullptr, 'm' },
        { "help", no_argument, nullptr, 'h' },
        { nullptr, 0, nullptr, '\0' }};
    
    while ((option = getopt_long(argc, argv, "m:h", longOpts, &option_index)) != -1) {
        switch (option) {
                
            case 'h':
                
                std::cerr << "This program simulates an on-campus drone delivery service.\n"
                
                << "There are two types of drones (Drone Type I and Drone Type II), and \n"
                << "three types of clients (A, B, and C). The program will aim to find the\n"
                << "shortest route distancewise to be able to service all locations across\n"
                << "campus.\n"
                << "Usage: \'./drone\n"
                <<                      "\t[--help | -h]\n"
                <<                      "\t[--mode | -m] <TYPE (either \"MST\", \"FASTTSP\", or \"OPTTSP\">\n";
                
                exit(0);
                
            case 'm':
                
                if (strcmp(optarg, "MST") == 0) { // MST
                    
                    mode = 'M';
                    
                }
                
                else if (strcmp(optarg, "FASTTSP") == 0) { // FASTTSP
                    
                    mode = 'F';
                    
                }
                
                else if (strcmp(optarg, "OPTTSP") == 0) { // OPTTSP
                    
                    mode = 'O';
                    
                }
                
                else { // Invalid command-line argument
                    
                    std::cerr << "Error: Invalid command line arguments. \"mode\" must be either: "
                    << "\"MST\", \"FASTTSP\", or \"OPTTSP\". Program terminating\n";
                    
                }
                
                break;
                
            default:
                
                std::cerr << "Error: Invalid command line arguments. Program terminating\n";
                
                exit(1);
                
        }
        
    }
    
}
        
char Drone::get_mode() {
    
    return mode;
    
}
        
double Drone::get_distance(Location &l1, Location &l2) {
    
    LocationType t1 = l1.get_location_type();
    LocationType t2 = l2.get_location_type();
    
    // Unreachable (one is normal and one is medical); return infinity
    if ((t1 == LocationType::Medical && t2 == LocationType::Normal) ||
        (t2 == LocationType::Medical && t1 == LocationType::Normal)) {
        
        return std::numeric_limits<double>::infinity();
        
    }
    
    //               ____________________________
    // Formula:     /          2              2
    //             /  (X2 - X1)   +  (Y2 - Y1)
    //           \/
    
    double x1 = static_cast<double>(l1.get_x_coord());
    double y1 = static_cast<double>(l1.get_y_coord());
    
    double x2 = static_cast<double>(l2.get_x_coord());
    double y2 = static_cast<double>(l2.get_y_coord());
    
    double distance = pow((x2 - x1), 2) + pow((y2 - y1), 2);
    
    distance = sqrt(distance);
    
    return distance;
    
}

// ----------------------------------------------------------------------------
//                    PART A: MST
// ----------------------------------------------------------------------------

void Drone::run_MST() {
    
    read_input();
    
    prim_algorithm();
    
    MST_print();
    
}


// reads in number of locations, initializes a member variables
// reads in locations and adds them to a vector
void Drone::read_input() {
    
    int num_locations_in, x_in, y_in;
    
    std::cin >> num_locations_in;
    
    num_locations = num_locations_in;
    
    v_locations.reserve(static_cast<size_t>(num_locations));
    
    for (int i = 0; i < num_locations; i++) {
        
        std::cin >> x_in >> y_in;
        
        Location l_in(x_in, y_in, i, mode);
        
        v_locations.push_back(l_in);
        
    }
    
}

void Drone::prim_algorithm() {
    
    // first location to start tree
    Location first = v_locations[0];

    prim_initialize_vectors(first, 0);
    
    int count = 1;
    
    // Algorithm
    
    // while not all locations have been visited
    while (count != num_locations) {
        
        size_t next_location_index = find_closest_location();
        
        prim_algorithm_update(v_locations[next_location_index], next_location_index);
        
        count++;
        
    }
    
}

void Drone::prim_initialize_vectors(Location &first_location, size_t first_location_index) {
    
    // Initializing vector prim_parents
    std::vector<Location> temp_parents(static_cast<size_t>(num_locations), first_location);
    prim_parents.swap(temp_parents);
    
    // Initializing vector prim_distances
    std::vector<double> temp_distances(static_cast<size_t>(num_locations));
    
    // Filling vector with distance from each location to first location/first parent
    for (size_t i = 0; i < static_cast<size_t>(num_locations); i++) {
        
        temp_distances[i] = get_distance(first_location, v_locations[i]);
        
    }
    
    prim_distances.swap(temp_distances);
    
    // Initializing vector prim_visited
    std::vector<bool> temp_visited(static_cast<size_t>(num_locations), false);
    
    // setting first location to visited
    temp_visited[first_location_index] = true;
    
    prim_visited.swap(temp_visited);
    
    return;
    
}

size_t Drone::find_closest_location() {
    
    double min_distance = std::numeric_limits<double>::infinity();
    
    int index = -1;
    
//    for (int i = 0; i < num_locations; i++) {
//
//        if (prim_distances[static_cast<size_t>(i)] < min_distance) {
//
//            // unvisited location
//            if (prim_visited[static_cast<size_t>(i)] == false) {
//
//                // new minimum distance
//                min_distance = prim_distances[static_cast<size_t>(i)];
//
//                index = i;
//
//            }
//
//        }
//
//    }
    
    for (int i = 0; i < static_cast<int>(prim_distances.size()); i++) {
        
        if (prim_distances[static_cast<size_t>(i)] < min_distance) {
            
            // unvisited location
            if (prim_visited[static_cast<size_t>(i)] == false) {
                
                // new minimum distance
                min_distance = prim_distances[static_cast<size_t>(i)];
                
                index = i;
                
            }
            
        }
        
    }
    
    // DEBUG
    if ((min_distance == std::numeric_limits<double>::infinity()) || (index == -1)) {
        
        std::cerr << "Error: No closest location found. Program terminating\n";
        
        exit(1);
        
    }
    
    return static_cast<size_t>(index);
    
}

void Drone::prim_algorithm_update(Location &next_location, size_t next_location_index) {
    
    // added to the tree
    prim_visited[next_location_index] = true;
    
    for (size_t i = 0; i < static_cast<size_t>(num_locations); i++) {
        
        // same location as next location
        if (i == next_location_index) {
            
            continue;
            
        }
        
        // Only looking at locations that are not part of the map
        if (prim_visited[i] == false) {
            
            double temp_distance = get_distance(next_location, v_locations[i]);
            
            // if (distance between this location and next location) is less than (distance to current parent)
            if (temp_distance < prim_distances[i]) {
                
                // New parent, update distance
                prim_parents[i] = next_location;
                
                prim_distances[i] = temp_distance;
                
            }
            
        }
        
    }
    
}

double Drone::MST_get_total_distance() {
    
    double total_weight = 0;
    
    // Finding total distances
    for (size_t i = 0; i < prim_distances.size(); i++) {
        
        // DEBUG
        if (prim_distances[i] == std::numeric_limits<double>::infinity()) {
            
            std::cerr << "Error: After MST Tree was constructed, found an edge with length INFINITY. Program terminating\n";
            
            exit(1);
            
        }
        
        total_weight += prim_distances[i];
        
    }
    
    return total_weight;
    
    
}

void Drone::MST_print() {

    double total_weight = MST_get_total_distance();
    
//    // Finding total distances
//    for (size_t i = 0; i < prim_distances.size(); i++) {
//
//        // DEBUG
//        if (prim_distances[i] == std::numeric_limits<double>::infinity()) {
//
//            std::cerr << "Error: After MST Tree was constructed, found an edge with length INFINITY. Program terminating\n";
//
//            exit(1);
//
//        }
//
//        total_weight += prim_distances[i];
//
//    }
    
    std::cout << total_weight << "\n";
    
    // Skipping first location (it is its own parent)
    for (size_t i = 1; i < static_cast<size_t>(num_locations); i++) {
        
        Location this_location = v_locations[i];
        Location parent_location = prim_parents[i];
        
        if (this_location.get_location_num() < parent_location.get_location_num()) {
            
            std::cout << this_location.get_location_num() << " " << parent_location.get_location_num() << "\n";
            
        }
        
        else {
            
            std::cout << parent_location.get_location_num() << " " << this_location.get_location_num() << "\n";
            
        }
        
    }
    
}

// ----------------------------------------------------------------------------
//                    PART B: FASTTSP
// ----------------------------------------------------------------------------

void Drone::run_FASTTSP() {
    
    read_input();
    
    double total_distance = 0;
    
    FAST_initialize_vectors(0, 1, 2, total_distance);
    
    FAST_arbitrary_insert_algorithm(total_distance);
    
    FAST_print(total_distance);
    
}

void Drone::FAST_initialize_vectors(size_t first_index, size_t second_index, size_t third_index, double &total_distance) {
    
    // +1 accounts for 0 (looping back to first index. Ex: 0-> 1-> 2-> 0)
    FAST_path.reserve(static_cast<size_t>(num_locations + 1));
    
    FAST_path.push_back(first_index);
    FAST_path.push_back(second_index);
    FAST_path.push_back(third_index);
    FAST_path.push_back(first_index);
    
    total_distance += get_distance(v_locations[first_index], v_locations[second_index]) +
    get_distance(v_locations[second_index], v_locations[third_index]) +
    get_distance(v_locations[third_index], v_locations[first_index]);
    
}

void Drone::FAST_arbitrary_insert_algorithm(double &total_distance) {
    
    // starting at index 3 (4th Location)
    // looping through rest of locations
    for (size_t i = 3; i < static_cast<size_t>(num_locations); i++) {
        
        double min_distance_change = std::numeric_limits<double>::infinity();
        
        // should insert after index found
        // ex: (0, 1, 2, 3) -> insert between 1 and 2
        // => index where this is found is 1 (between 1 and 2)
        // => insert at index 2 (index 1 + 1 = 2)
        
        // size_t index_to_insert;
        
        // looping through current path
        // -1: vector path already accounts for last/first comparison; don't check last value
        // (0, 1, 2, 0) -> only check 0 (0 -> 1), 1 (1 -> 2), 2 (2 -> 0)
        
        // iterators for vector.insert()
        auto it = FAST_path.begin();
        auto it_insert_index = it;
        
        for (size_t j = 0; j < FAST_path.size() - 1; j++) {
            
            // +1 is for looking at this location and next location
            
            double distance_change = FAST_distance_change(j, (j + 1), i);
            
            // shorter distance than current best
            if (distance_change < min_distance_change) {
                
                min_distance_change = distance_change;
                
                //index_to_insert = j + 1;
                
                it_insert_index = it;
                it_insert_index++;
                
            }
            
            it++;
            
        }
        
        // distance added from inserting location into path
        total_distance += min_distance_change;
        
        // i = index of location being inserted
        // 0 will always be the last element in FAST_path
        FAST_path.insert(it_insert_index, i);
        
    }
    
}

double Drone::FAST_distance_change(size_t first_index, size_t second_index, size_t new_index) {
    
    // v_locations(0, 1, 2, ...) [path(1, 4, 2, 3, ...)]
    Location first = v_locations[FAST_path[first_index]];
    Location second = v_locations[FAST_path[second_index]];
    Location new_location = v_locations[new_index];
    
    //
    // Formula: change in distance = d(i, k) + d(k, j) - d(i, j)
    //
    //    Location i: first Location
    //
    //    Location j: second Location
    //
    //    Location k: location being inserted into the path
    //
    
    double distance_change = get_distance(first, new_location) + get_distance(new_location, second) - get_distance(first, second);
    
    return distance_change;
    
}

void Drone::FAST_print(double total_distance) {
    
    // popping 0 at the back
    FAST_path.pop_back();
    
    std::cout << total_distance << "\n";
    
    // printing path
    for (size_t i = 0; i < FAST_path.size(); i++) {
        
        std::cout << FAST_path[i] << " ";
        
    }
    
}


// ----------------------------------------------------------------------------
//                    PART C: OPTTSP
// ----------------------------------------------------------------------------

void Drone::run_OPTTSP() {
    
    read_input();
    
    OPT_initialize();
    
    
//    // DEBUG
//    std::cout << OPT_best_distance << "\n";
//    for (size_t i = 0; i < FAST_path.size(); i++) {
//
//        std::cout << FAST_path[i] << " ";
//
//    }
//    std::cout << "\n";
    
    genPerms(1);
    
    OPT_print();
    
}

void Drone::OPT_initialize() {
    
    //std::vector<bool> temp_visited(static_cast<size_t>(num_locations), false);
    //OPT_visited.swap(temp_visited);
    
    // initializes:
    
    //     OPT_best_path
    //     OPT_best_distance
    OPT_FASTTSP_helper();
    
    OPT_path.resize(static_cast<size_t>(num_locations));
    
    // 0, 1, 2, 3...
    for (size_t i = 0; i < static_cast<size_t>(num_locations); i++) {

        OPT_path[i] = i;

    }
    
    OPT_current_distance = 0;
    
    // Location 0 is visited first
    //OPT_visited[0] = true;
    
}

// initializes:
//     OPT_path
//     OPT_best_path
//     OPT_best_distance
void Drone::OPT_FASTTSP_helper() {
    
    double total_distance = 0;
    
    FAST_initialize_vectors(0, 1, 2, total_distance);
    
    FAST_arbitrary_insert_algorithm(total_distance);
    
    OPT_best_distance = total_distance;
    
    
    // Popping 0 at the back of vector
    FAST_path.pop_back();
    
    OPT_best_path = FAST_path;
    
}

void Drone::genPerms(size_t permLength) {
    
    // DEBUG
    
    if (permLength == 25) {

        std::cout << "DEBUG\n";

    }
    
    if (permLength == OPT_path.size()) {
        
        // add closing edge
        // check/update best distance/path
        // subtract closing edge
        
        // closing edge
        double closing_edge = get_distance(v_locations[OPT_path[0]], v_locations[OPT_path[permLength - 1]]);
        
        
//        //DEBUG:
//
//        if (OPT_current_distance + closing_edge > 354.2 && OPT_current_distance + closing_edge < 354.3) {
//
//            std::cout << "DEBUG\n";
//
//            int i = 0;
//            i++;
//
//        }
        
        OPT_current_distance += closing_edge;
        
        // Better than previous distance; update best distance
        if (OPT_current_distance < OPT_best_distance) {
            
            OPT_best_distance = OPT_current_distance;
            
            OPT_best_path = OPT_path;
            
        }
        
        OPT_current_distance -= closing_edge;
        
        return;
        
    } // if
    if (is_promising(permLength) == false) {
        
        return;
        
    }
    
    for (size_t i = permLength; i < OPT_path.size(); ++i) {
        
        std::swap(OPT_path[permLength], OPT_path[i]);
        
        OPT_current_distance += get_distance(v_locations[OPT_path[permLength]], v_locations[OPT_path[permLength - 1]]);
        
        genPerms(permLength + 1);
        
        OPT_current_distance -= get_distance(v_locations[OPT_path[permLength]], v_locations[OPT_path[permLength - 1]]);
        
        std::swap(OPT_path[permLength], OPT_path[i]);
        
        //OPT_visited[OPT_path[i]] = true;
        
    } // for
    
} // genPerms()

bool Drone::is_promising(size_t permLength) {
    
    // if there is 4 or less unvisited vertices
    if (OPT_path.size() - permLength <= 5) {
        
        return true;
        
    }
    
    // Making a MST out of unvisited Locations
    std::vector<size_t> unvisited;
    unvisited.reserve(static_cast<size_t>(num_locations) - permLength);
    
    for (size_t i = permLength; i < OPT_path.size(); i++) {
        
        unvisited.push_back(i);
        
    }
    
//    for (size_t i = 0; i < OPT_visited.size(); i++) {
//
//        if (OPT_visited[i] == false) {
//
//            // permLength is already at the front of index
//            if (i != permLength) {
//
//                unvisited.push_back(i);
//
//            }
//
//        }
//
//    }
    
    //OPT_modified_prim_initialize_vectors(v_locations[unvisited[0]], 0, unvisited);
    
    OPT_modified_prim_algorithm(unvisited);
    
    // MST created
    
//    // Edge from MST to beginning of path (closing edge)
//    double connecting_edge = get_distance(v_locations[OPT_path[permLength]], v_locations[OPT_path[0]]);
    
    //size_t zero_connecting_index = 0, last_connecting_index = 0;
    double zero_distance = std::numeric_limits<double>::infinity(), last_distance = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < unvisited.size(); i++) {
        
        // Distance from first Location in path to this unvisited locaiton
        double temp_zero = get_distance(v_locations[OPT_path[unvisited[i]]], v_locations[OPT_path[0]]);
        
        if (temp_zero < zero_distance) {
            
            zero_distance = temp_zero;
            
        }
        
        // Distance from last fixed Location in path to this unvisited locaiton
        double temp_last = get_distance(v_locations[OPT_path[unvisited[i]]], v_locations[OPT_path[permLength - 1]]);
        
        if (temp_last < last_distance) {
            
            last_distance = temp_last;
            
        }
        
    }
    
    // Estimated distance + distance traveled already + connecting_edge
    double lower_bound = MST_get_total_distance() + OPT_current_distance + zero_distance + last_distance;
    
    // DEBUG:
//    std::cout << MST_get_total_distance() << "\n";
//
//    for (size_t i = 0; i < unvisited.size(); i++) {
//
//        std::cout << v_locations[unvisited[i]].get_x_coord() << " " << v_locations[unvisited[i]].get_y_coord() << "\n";
//
//    }
    
//    for (size_t i = 0; i < prim_distances.size() - 1; i++) {
//
//        Location this_location = v_locations[unvisited[i]];
//        Location parent_location = prim_parents[unvisited[i]];
//
//        if (this_location.get_location_num() < parent_location.get_location_num()) {
//
//            std::cout << unvisited[i] << " " << parent_location.get_location_num() - 1 << "\n";
//
//        }
//
//        else {
//
//            std::cout << parent_location.get_location_num() - 1 << " " << unvisited[i] << "\n";
//
//        }
//
//    }
    
    
    // clear MST to be used again
    
    OPT_reset_prim();
    
    // keep searching this path
    if (lower_bound < OPT_best_distance) {
        
        return true;
        
    }
    
    else {
        
        return false;
        
    }
    
    
}


void Drone::OPT_modified_prim_algorithm(std::vector<size_t> &unvisited_locations) {
    
    // first location to start tree
    Location first = v_locations[unvisited_locations[0]];
//
    OPT_modified_prim_initialize_vectors(first, 0, unvisited_locations);
    
    size_t count = 1;
    
    while (count != unvisited_locations.size()) {
        
        size_t next_location_index = find_closest_location();
        
        OPT_modified_prim_update(v_locations[unvisited_locations[next_location_index]], next_location_index, unvisited_locations);
        
        count++;
        
    }
    
}

void Drone::OPT_modified_prim_initialize_vectors(Location &first_location, size_t first_location_index, std::vector<size_t> &unvisited_locations) {
    
    // Initializing vector prim_parents
    std::vector<Location> temp_parents(static_cast<size_t>(unvisited_locations.size()), first_location);
    prim_parents.swap(temp_parents);
    
    // Initializing vector prim_distances
    std::vector<double> temp_distances(unvisited_locations.size());
    
    // Filling vector with distance from each location to first location/first parent
    for (size_t i = 0; i < unvisited_locations.size(); i++) {
        
        temp_distances[i] = get_distance(first_location, v_locations[unvisited_locations[i]]);
        
    }
    
    prim_distances.swap(temp_distances);
    
    // Initializing vector prim_visited
    std::vector<bool> temp_visited(unvisited_locations.size(), false);
    
    // setting first location to visited
    temp_visited[first_location_index] = true;
    
    prim_visited.swap(temp_visited);
    
    return;
    
}

void Drone::OPT_modified_prim_update(Location &next_location, size_t next_location_index, std::vector<size_t> &unvisited_locations) {
    
    // added to the tree
    prim_visited[next_location_index] = true;
    
    for (size_t i = 0; i < unvisited_locations.size(); i++) {
        
        // same location as next location
        if (i == next_location_index) {
            
            continue;
            
        }
        
        // Only looking at locations that are not part of the map
        if (prim_visited[i] == false) {
            
            double temp_distance = get_distance(next_location, v_locations[unvisited_locations[i]]);
            
            // if (distance between this location and next location) is less than (distance to current parent)
            if (temp_distance < prim_distances[i]) {
                
                // New parent, update distance
                prim_parents[i] = next_location;
                
                prim_distances[i] = temp_distance;
                
            }
            
        }
        
    }
    
}

void Drone::OPT_reset_prim() {
    
    prim_parents.clear();
    
    prim_distances.clear();
    
    prim_visited.clear();
    
}

void Drone::OPT_print() {
    
//    double closing_edge = get_distance(v_locations[OPT_best_path.front()], v_locations[OPT_best_path.back()]);
//
//    OPT_best_distance += closing_edge;
    
    std::cout << OPT_best_distance << "\n";
    
    for (size_t i = 0; i < OPT_best_path.size(); i++) {
        
        std::cout << OPT_best_path[i] << " ";
        
    }
    
//    // DEBUG
//
//    std::vector<size_t> debug;
//
//    for (size_t  i = 13; i < OPT_best_path.size(); i++) {
//
//        debug.push_back(OPT_best_path[i]);
//
//    }
//
//    OPT_modified_prim_algorithm(debug);
//
//    std::cout << MST_get_total_distance() << "\n";
//
//    for (size_t i = 0; i < debug.size(); i++) {
//
//        std::cout << v_locations[debug[i]].get_x_coord() << " " << v_locations[debug[i]].get_y_coord() << "\n";
//
//    }
    
}
