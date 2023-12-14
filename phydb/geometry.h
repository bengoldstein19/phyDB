/*******************************************************************************
 *
 * Copyright (c) 2023 Benjamin Goldstein 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 ******************************************************************************/
#ifndef PHYDB_GEOMETRY_H_
#define PHYDB_GEOMETRY_H_

#define BIN_WIDTH 750
#define NUM_BINS_NEIGHBORHOOD 2

#include "datatype.h"

namespace phydb {

    // class declarations for type awareness
    class WireSegment;

    class Resistor;

    template<typename T>
    class UniformPartition;

    class Geometry;

    class Resistor {
        public:
            std::string getNodeId1() const { return _node_id1; }
            void setNodeId1(std::string node_id1) { _node_id1 = node_id1; }

            std::string getNodeId2() const { return _node_id2; }
            void setNodeId2(std::string node_id2) { _node_id2 = node_id2; }

            std::string getNetName();

            std::string getMaterial() const { return _material; }
            double getLength() const { return _length; }
            void setLength(double l) { _length = l; }

            double getWidth() const { return _width; }
            double getArea() const { return _area; }

            Point2D<double> getP1() const { return _p1; }
            void setP1(Point2D<double> pt) { _p1 = pt; }
            Point2D<double> getP2() const { return _p2; }
            void setP2(Point2D<double> pt) { _p2 = pt; }

            WireSegment *getOwnerSegment() { return _owner_segment; }

            Resistor(std::string node_id1, std::string node_id2, std::string material, double length = -1, double width = -1, double area = -1, Point2D<double> p1 = Point2D<double>(), Point2D<double> p2 = Point2D<double>(), WireSegment *owner_segment = nullptr);

            void print(std::ostream &s);

            Resistor(const Resistor &res) :
                _node_id1(res._node_id1),
                _node_id2(res._node_id2),
                _material(res._material),
                _length(res._length),
                _width(res._width),
                _area(res._area),
                _p1(res._p1),
                _p2(res._p2),
                _owner_segment(res._owner_segment)
            {
                std::cout << "this shouldnt happen i dont think..." << std::endl;
            }

        private:
            std::string _node_id1;
            std::string _node_id2;
            std::string _material;
            double _length;
            double _width;
            double _area;
            Point2D<double> _p1;
            Point2D<double> _p2;
            WireSegment *_owner_segment;
    };

    class Capacitor {
        public:
            std::string getNodeId1() const { return _node_id1; }
            std::string getNodeId2() const { return _node_id2; }
            std::string getMaterial() const { return _material; }
            double getOverlapLength() const { return _overlap_length; }
            double getDistance() const { return _distance; }
            void print(std::ostream &s);

            Capacitor(std::string node_id1, std::string node_id2, std::string material, double overlap_length, double distance) :
                _node_id1(node_id1),
                _node_id2(node_id2),
                _material(material),
                _overlap_length(overlap_length),
                _distance(distance) {}

        private:
            std::string _node_id1;
            std::string _node_id2;
            std::string _material;
            double _overlap_length;
            double _distance;


    };

    class WireSegment {
        public:
            Rect2D<double> getRect() const { return _rect; }
            std::string getNetName() const { return _net_name; }
            std::string getLayerName() const { return _layer_name; }
            int getSegmentNumber() const { return _seg_num; }

            void addHorizontalConnection(WireSegment *seg_ptr) { _horizontal_connections.emplace_back(seg_ptr); }
            void addVerticalConnection(WireSegment *seg_ptr) { _vertical_connections.emplace_back(seg_ptr); }
            void addResistor(Resistor *res) { _resistors.emplace_back(res); }

            std::vector<WireSegment *> &getHorizontalConnections() { return _horizontal_connections; }
            std::vector<WireSegment *> &getVerticalConnections() { return _vertical_connections; }
            std::vector<Resistor *> &getResistors() { return _resistors; }

            Point2D<double> getP1() { return _p1; }
            Point2D<double> getP2() { return _p2; }
            

            WireSegment(Rect2D<double> rect, std::string net_name, std::string layer_name, int seg_num, Point2D<double> p1 = Point2D<double>(), Point2D<double> p2 = Point2D<double>()) :
                _rect(rect),
                _net_name(net_name),
                _layer_name(layer_name),
                _seg_num(seg_num),
                _horizontal_connections(std::vector<WireSegment *>()),
                _vertical_connections(std::vector<WireSegment *>()),
                _p1(p1),
                _p2(p2),
                _resistors(std::vector<Resistor *>()) {}

            WireSegment(const WireSegment &w) = default;



        private:
            Rect2D<double> _rect; // rectangle defining shape of wire segment
            std::string _net_name; // name of net wire segment is part of
            std::string _layer_name; // name of layer wire segment is on
            int _seg_num; // unique integer identifier of segment within net, sequentially numbered
            std::vector<WireSegment *> _horizontal_connections; // side to side rectangle connections, from partitioned wire segs
            std::vector<WireSegment *> _vertical_connections; // inter-layer/vertical connections from vias
            Point2D<double> _p1; // coordinates for resistance generation
            Point2D<double> _p2;
            std::vector<Resistor *> _resistors;

    };

    // type T must have getRect member functoin that returns Rect2D<double>
    template<typename T>
    class UniformPartition {
        public:
            UniformPartition(double partition_size) :
                _partition_pair_to_segs(std::map<std::pair<int, int>, std::vector<T *>>()),
                _partition_size(partition_size) { }

            void add(T *elt);
            std::pair<int, int> getPartitionId(Point2D<double> pt) const { return std::pair<int, int>(static_cast<int>(pt.x / _partition_size), static_cast<int>(pt.y / _partition_size)); }
            std::vector<T *> getElementsByPartition(std::pair<int, int> partition) const;

        private:
            std::map<std::pair<int, int>, std::vector<T *>> _partition_pair_to_segs;
            double _partition_size;

    };

    class Geometry {
        public:
            Geometry(double default_partition_size = BIN_WIDTH, int num_bins_neighborhood = NUM_BINS_NEIGHBORHOOD) :
                _net_to_segs(std::map<std::string, std::vector<std::unique_ptr<WireSegment>>>()),
                _layer_to_partitioned_segs(std::map<std::string, UniformPartition<WireSegment>>()),
                _default_partition_size(default_partition_size),
                _num_bins_neighborhood(num_bins_neighborhood),
                _net_to_num_nodes(std::map<std::string, unsigned long>()),
                _resistor_network(std::vector<std::unique_ptr<Resistor>>())
                {}

            WireSegment *addSegmentToNet(std::string net,  WireSegment &seg);
            WireSegment *addSegmentToLayer(std::string layer, WireSegment *seg_ptr);

            WireSegment *addWireSegment(WireSegment &seg) { return addSegmentToLayer(seg.getLayerName(), addSegmentToNet(seg.getNetName(), seg)); }

            std::vector<std::unique_ptr<WireSegment>> &getSegmentsOfNet(std::string net);

            std::vector<WireSegment *> getOtherNetsNearbySegments(WireSegment *seg_ptr);

            std::string generateNodeID(std::string net);

            void generateRCNetwork() { _populateResistorNetwork(); _populateCapacitanceNetwork(); }

            void printRCNetwork(std::ostream &stream);

        private:
            std::map<std::string, std::vector<std::unique_ptr<WireSegment>>> _net_to_segs;
            std::map<std::string, UniformPartition<WireSegment>> _layer_to_partitioned_segs;
            double _default_partition_size;
            double _num_bins_neighborhood;
            std::map<std::string, unsigned long> _net_to_num_nodes; // for node id generation (format net{nodenum})
            std::vector<std::unique_ptr<Resistor>> _resistor_network;
            std::vector<std::unique_ptr<Capacitor>> _capacitor_network;

            //resistor/capacitor generation helper functions
            void _populateResistorNetwork();
            void _populateCapacitanceNetwork();
            void _handleContains(WireSegment *super_seg, WireSegment *sub_seg); // helper function to handle total segment containment
            void _handleOverlap(WireSegment *seg1, WireSegment *seg2); // helper function to handle segment overlap

            std::string _splitResistorAtPt(Resistor *res, Point2D<double> sub_seg_pt); // handles splitting resistor, returns new node id
    };

}

#endif //PHYDB_GEOMETRY_H_
