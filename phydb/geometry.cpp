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

#include <set>

#include "geometry.h"

namespace phydb {

    bool rectContains(Rect2D<double> r, Point2D<double> p) {
        return p.x >= r.ll.x && p.x <= r.ur.x && p.y >= r.ll.y && p.y <= r.ur.y;
    }

    std::string Resistor::getNetName() { return _owner_segment->getNetName(); }

    Resistor::Resistor(std::string node_id1, std::string node_id2, std::string material, double length, double width, double area, Point2D<double> p1, Point2D<double> p2, WireSegment *owner_segment) :
                _node_id1(node_id1),
                _node_id2(node_id2),
                _material(material),
                _length(length),
                _width(width),
                _area(area),
                _p1(p1),
                _p2(p2),
                _owner_segment(owner_segment) {
        if (_owner_segment) {
            _owner_segment->addResistor(this);
        }
    }

    void Resistor::print(std::ostream &s) {
        if (_area == -1) {
            s << "Resistor<node1='"
              << _node_id1 
              << "', node2='"
              << _node_id2
              << "', length="
              << _length
              << ", width="
              << _width
              << ", layer="
              << _material
              << ", segment-id='"
              << _owner_segment->getNetName()
              << ":"
              << _owner_segment->getSegmentNumber()
              << "'>"
              << std::endl;
        } else {
            s << "VerticalResistor<lower-node='"
              << _node_id1 
              << "', upper-node='"
              << _node_id2
              << "', cross-sectional-area="
              << _area
              << ", layer="
              << _material
              << ", segment-id='"
              << _owner_segment->getNetName()
              << ":"
              << _owner_segment->getSegmentNumber()
              << "'>"
              << std::endl;
        }
    }

    void Capacitor::print(std::ostream &s) {
        s << "Capacitor<node1='"
          << _node_id1
          << "', node2='"
          << _node_id2
          << "', overlap-length="
          << _overlap_length
          << ", distance="
          << _distance
          << ">"
          << std::endl;
    }

    void Geometry::_populateResistorNetwork() {
        if (_resistor_network.size() != 0) {
            std::cout << "WEIRD: INPUT VECTOR NON EMPTY" << std::endl;
            return;
        }

        // (1) for each centerline segment generate an internal resistor
        for (std::map<std::string, std::vector<std::unique_ptr<WireSegment>>>::iterator it = _net_to_segs.begin() ; it != _net_to_segs.end() ; it++) {
            for (std::unique_ptr<WireSegment> &seg_ptr : it->second) {
                Point2D<double> p1 = seg_ptr->getP1();
                Point2D<double> p2 = seg_ptr->getP2();
                if (p1.x != p2.x || p1.y != p2.y) { // segment a non via rect
                    double length = p1.x != p2.x ? abs(p1.x - p2.x) : abs(p1.y - p2.y);
                    Rect2D<double> seg_rect = seg_ptr->getRect();
                    double width = p1.x != p2.x ? seg_rect.ur.y - seg_rect.ll.y : seg_rect.ur.x - seg_rect.ll.x;
                    std::unique_ptr<Resistor> resistor(new Resistor(generateNodeID(it->first), generateNodeID(it->first), seg_ptr->getLayerName(), length, width, -1, p1, p2, seg_ptr.get()));
                    _resistor_network.emplace_back(std::move(resistor));
                }
            }
        }

        // (2) wire up horizontal connections using "escape resistors", vertical connections by area resistors
        for (std::map<std::string, std::vector<std::unique_ptr<WireSegment>>>::iterator it = _net_to_segs.begin() ; it != _net_to_segs.end() ; it++) {
            for (std::unique_ptr<WireSegment> &seg_ptr : it->second) {
                std::vector<WireSegment *> &horizontal_connections = seg_ptr->getHorizontalConnections();
                if (horizontal_connections.size() != 0) {
                    for (WireSegment *prev_seg_ptr : horizontal_connections) { // pretty sure there will only ever be 1
                        Point2D<double> p1 = prev_seg_ptr->getP2();
                        Point2D<double> p2 = seg_ptr->getP1();

                        double length = p1.x != p2.x ? abs(p1.x - p2.x) : abs(p1.y - p2.y);
                        Rect2D<double> seg_rect = seg_ptr->getRect();
                        double width = p1.x != p2.x ? seg_rect.ur.y - seg_rect.ll.y : seg_rect.ur.x - seg_rect.ll.x;
                        std::unique_ptr<Resistor> resistor(new Resistor(generateNodeID(it->first), generateNodeID(it->first), seg_ptr->getLayerName(), length, width, -1, p1, p2, prev_seg_ptr));
                        _resistor_network.emplace_back(std::move(resistor));
                    }
                }

                std::vector<WireSegment *> &vertical_connections = seg_ptr->getVerticalConnections();
                for (WireSegment *prev_seg_ptr : vertical_connections) {
                    // generate resistor for current segment if it doesn't yet exist, otherwise just get "bottom" node id

                    std::vector<Resistor *> seg_resistors = seg_ptr->getResistors();
                    std::string seg_bottom_node;
                    if (seg_resistors.size() == 0) {
                        Rect2D<double> seg_rect = seg_ptr->getRect();
                        double area = (seg_rect.ur.x - seg_rect.ll.x) * (seg_rect.ur.y - seg_rect.ll.y);
                        seg_bottom_node = generateNodeID(it->first);
                        std::unique_ptr<Resistor> resistor(new Resistor(seg_bottom_node, generateNodeID(it->first), seg_ptr->getLayerName(), -1, -1, area, seg_ptr->getP1(), seg_ptr->getP2(), seg_ptr.get()));
                        _resistor_network.emplace_back(std::move(resistor));
                    } else {
                        seg_bottom_node = seg_resistors[0]->getNodeId1();
                    }

                    // generate resistor for prev segment if doesn't exist with top node = seg's bottom node, otherwise set prev's top node equal to seg's bottom
                    std::vector<Resistor *> prev_resistors = prev_seg_ptr->getResistors();
                    if (prev_resistors.size() == 0) {
                        Rect2D<double> prev_rect = prev_seg_ptr->getRect();
                        double area = (prev_rect.ur.x - prev_rect.ll.x) * (prev_rect.ur.y - prev_rect.ll.y);
                        std::unique_ptr<Resistor> resistor(new Resistor(generateNodeID(prev_seg_ptr->getNetName()),
                                                                        seg_bottom_node,
                                                                        prev_seg_ptr->getLayerName(),
                                                                        -1,
                                                                        -1,
                                                                        area,
                                                                        prev_seg_ptr->getP1(),
                                                                        prev_seg_ptr->getP2(),
                                                                        prev_seg_ptr));
                        _resistor_network.emplace_back(std::move(resistor));
                    } else {
                        // coalesce nodes if resistor already exists
                        prev_resistors[0]->setNodeId2(seg_bottom_node);
                    }
                }
            }
        }

        // (3) pairwise operations: combine vias into connected wires, non centerline connected wires. May not be dependable depending on net size
        for (std::map<std::string, std::vector<std::unique_ptr<WireSegment>>>::iterator it = _net_to_segs.begin() ; it != _net_to_segs.end() ; it++) {
            for (long i = 0; i < static_cast<long>(it->second.size()) - 1; i++) {
                for (unsigned long j = i + 1; j < it->second.size(); j++) {
                    WireSegment *seg = it->second[i].get();
                    WireSegment *other_seg = it->second[j].get();

                    if (seg->getLayerName() != other_seg->getLayerName()) continue; // only concerned with pairs that share layer

                    Rect2D<double> seg_rect = seg->getRect();
                    Rect2D<double> other_seg_rect = other_seg->getRect();

                    if (!rectContains(seg_rect, other_seg_rect.ll) && !rectContains(seg_rect, other_seg_rect.ur)) continue; // skip pairs with no overlap

                    if (rectContains(seg_rect, other_seg_rect.ll) && rectContains(seg_rect, other_seg_rect.ur)) { // other seg contained within seg, coalesce
                        _handleContains(seg, other_seg);
                    } else if (rectContains(other_seg_rect, seg_rect.ll) && rectContains(other_seg_rect, seg_rect.ur)) {
                        _handleContains(other_seg, seg);
                    } else if (rectContains(seg_rect, other_seg_rect.ll) || rectContains(seg_rect, other_seg_rect.ur)) {
                        _handleOverlap(seg, other_seg);
                    }
                }
            }
        }
    }

    std::string Geometry::generateNodeID(std::string net) {
        if (_net_to_num_nodes.find(net) == _net_to_num_nodes.end()) {
            _net_to_num_nodes[net] = 0;
        }

        unsigned long num_nodes = _net_to_num_nodes[net]++;
        return net + "{" + std::to_string(num_nodes) + "}";
        
    }

    std::string Geometry::_splitResistorAtPt(Resistor *res, Point2D<double> sub_seg_pt) {
        std::string new_id = generateNodeID(res->getNetName());
        Point2D<double> res_p1 = res->getP1();
        Point2D<double> res_p2 = res->getP2();

        std::string old_node_id2 = res->getNodeId2();
        
        res->setNodeId2(new_id);
        res->setP2(sub_seg_pt);
        double l = sub_seg_pt.x == res_p1.x ? abs(sub_seg_pt.y - res_p1.y) : abs(sub_seg_pt.x - res_p1.x);
        double old_l = res->getLength();
        res->setLength(l);

        std::unique_ptr<Resistor> resistor(new Resistor(new_id,
            old_node_id2,
            res->getMaterial(),
            old_l - l,
            res->getWidth(),
            -1,
            sub_seg_pt,
            res_p2,
            res->getOwnerSegment()
        ));
        _resistor_network.emplace_back(std::move(resistor));

        return new_id;
    }

    void Geometry::_handleOverlap(WireSegment *seg1, WireSegment *seg2) {
        if (seg1->getResistors().size() == 0 || seg2->getResistors().size() == 0) return;

        Point2D<double> seg1_p1 = seg1->getP1();
        Point2D<double> seg1_p2 = seg1->getP2();

        Point2D<double> seg2_p1 = seg2->getP1();
        Point2D<double> seg2_p2 = seg2->getP2();

        Rect2D<double> seg1_rect = seg1->getRect();
        Rect2D<double> seg2_rect = seg2->getRect();

        if (seg1_p1.x == seg1_p2.x && seg1_p1.x > std::min(seg2_p1.x, seg2_p2.x) && seg1_p1.x < std::max(seg2_p1.x, seg2_p2.x)) { // seg 1 vertical, cuts seg 2
            for (Resistor *res : seg2->getResistors()) {
                Point2D<double> res_p1 = res->getP1();
                Point2D<double> res_p2 = res->getP2();

                if (seg1_p1.x > std::min(res_p1.x, res_p2.x) && seg1_p1.x < std::max(res_p1.x, res_p2.x)) {
                    Point2D<double> split_pt(seg1_p1.x, res_p1.y);
                    std::string new_id = _splitResistorAtPt(res, split_pt);
                    double escape_jct_y = seg1_p1.y > res_p1.y ? seg2_rect.ur.y : seg2_rect.ll.y;
                    std::string escape_jct_id = generateNodeID(seg2->getNetName());

                    std::unique_ptr<Resistor> seg2_r_escape(new Resistor(new_id,
                        escape_jct_id,
                        seg2->getLayerName(),
                        abs(split_pt.y - escape_jct_y),
                        seg1_rect.ur.x - seg1_rect.ll.x,
                        -1,
                        split_pt,
                        Point2D<double>(split_pt.x, escape_jct_y),
                        seg2
                    ));
                    _resistor_network.emplace_back(std::move(seg2_r_escape));

                    double min_dst = -1;
                    std::string  min_dst_id = "";
                    Point2D<double> min_dst_pt;
                    for (Resistor *seg1_r : seg1->getResistors()) {
                        if (min_dst < 0 || abs(seg1_r->getP1().y - escape_jct_y) < min_dst) {
                            min_dst = abs(seg1_r->getP1().y - escape_jct_y);
                            min_dst_id = seg1_r->getNodeId1();
                            min_dst_pt = seg1_r->getP1();
                        }

                        if (min_dst < 0 || abs(seg1_r->getP2().y - escape_jct_y) < min_dst) {
                            min_dst = abs(seg1_r->getP2().y - escape_jct_y);
                            min_dst_id = seg1_r->getNodeId1();
                            min_dst_pt = seg1_r->getP2();
                        }
                    }
                    if (min_dst < 0) return;

                    std::unique_ptr<Resistor> seg1_r_escape(new Resistor(escape_jct_id,
                        min_dst_id,
                        seg1->getLayerName(),
                        min_dst,
                        seg1_rect.ur.x - seg1_rect.ll.x,
                        -1,
                        Point2D<double>(split_pt.x, escape_jct_y),
                        min_dst_pt,
                        seg1
                    ));
                    _resistor_network.emplace_back(std::move(seg1_r_escape));
                    return;
                }
            }

        } else if (seg1_p1.y == seg1_p2.y && seg1_p1.y > std::min(seg2_p1.y, seg2_p2.y) && seg1_p1.y < std::max(seg2_p1.y, seg2_p2.y)) { // seg 1 horizontal, cuts seg 2
            for (Resistor *res : seg2->getResistors()) {
                Point2D<double> res_p1 = res->getP1();
                Point2D<double> res_p2 = res->getP2();

                if (seg1_p1.y > std::min(res_p1.y, res_p2.y) && seg1_p1.y < std::max(res_p1.y, res_p2.y)) {
                    Point2D<double> split_pt(res_p1.x, seg1_p1.y);
                    std::string new_id = _splitResistorAtPt(res, split_pt);
                    double escape_jct_x = seg1_p1.x > res_p1.x ? seg2_rect.ur.x : seg2_rect.ll.x;
                    std::string escape_jct_id = generateNodeID(seg2->getNetName());

                    std::unique_ptr<Resistor> seg2_r_escape(new Resistor(new_id,
                        escape_jct_id,
                        seg2->getLayerName(),
                        abs(split_pt.x - escape_jct_x),
                        seg1_rect.ur.y - seg1_rect.ll.y,
                        -1,
                        split_pt,
                        Point2D<double>(escape_jct_x, split_pt.y),
                        seg2
                    ));
                    _resistor_network.emplace_back(std::move(seg2_r_escape));

                    double min_dst = -1;
                    std::string  min_dst_id = "";
                    Point2D<double> min_dst_pt;
                    for (Resistor *seg1_r : seg1->getResistors()) {
                        if (min_dst < 0 || abs(seg1_r->getP1().x - escape_jct_x) < min_dst) {
                            min_dst = abs(seg1_r->getP1().x - escape_jct_x);
                            min_dst_id = seg1_r->getNodeId1();
                            min_dst_pt = seg1_r->getP1();
                        }

                        if (min_dst < 0 || abs(seg1_r->getP2().x - escape_jct_x) < min_dst) {
                            min_dst = abs(seg1_r->getP2().x - escape_jct_x);
                            min_dst_id = seg1_r->getNodeId1();
                            min_dst_pt = seg1_r->getP2();
                        }
                    }
                    if (min_dst < 0) return;

                    std::unique_ptr<Resistor> seg1_r_escape(new Resistor(escape_jct_id,
                        min_dst_id,
                        seg1->getLayerName(),
                        min_dst,
                        seg1_rect.ur.y - seg1_rect.ll.y,
                        -1,
                        Point2D<double>(escape_jct_x, split_pt.y),
                        min_dst_pt,
                        seg1
                    ));
                    _resistor_network.emplace_back(std::move(seg1_r_escape));
                    return;
                }
            }

        } else if (seg2_p1.x == seg2_p2.x && seg2_p1.x > std::min(seg1_p1.x, seg1_p2.x) && seg2_p1.x < std::max(seg1_p1.x, seg1_p2.x)) { // seg 2 vertical, cuts seg 1
            for (Resistor *res : seg1->getResistors()) {
                Point2D<double> res_p1 = res->getP1();
                Point2D<double> res_p2 = res->getP2();

                if (seg2_p1.x > std::min(res_p1.x, res_p2.x) && seg2_p1.x < std::max(res_p1.x, res_p2.x)) {
                    Point2D<double> split_pt(seg2_p1.x, res_p1.y);
                    std::string new_id = _splitResistorAtPt(res, split_pt);
                    double escape_jct_y = seg2_p1.y > res_p1.y ? seg1_rect.ur.y : seg1_rect.ll.y;
                    std::string escape_jct_id = generateNodeID(seg1->getNetName());

                    std::unique_ptr<Resistor> seg1_r_escape(new Resistor(new_id,
                        escape_jct_id,
                        seg1->getLayerName(),
                        abs(split_pt.y - escape_jct_y),
                        seg2_rect.ur.x - seg2_rect.ll.x,
                        -1,
                        split_pt,
                        Point2D<double>(split_pt.x, escape_jct_y),
                        seg1
                    ));
                    _resistor_network.emplace_back(std::move(seg1_r_escape));

                    double min_dst = -1;
                    std::string  min_dst_id = "";
                    Point2D<double> min_dst_pt;
                    for (Resistor *seg2_r : seg2->getResistors()) {
                        if (min_dst < 0 || abs(seg2_r->getP1().y - escape_jct_y) < min_dst) {
                            min_dst = abs(seg2_r->getP1().y - escape_jct_y);
                            min_dst_id = seg2_r->getNodeId1();
                            min_dst_pt = seg2_r->getP1();
                        }

                        if (min_dst < 0 || abs(seg2_r->getP2().y - escape_jct_y) < min_dst) {
                            min_dst = abs(seg2_r->getP2().y - escape_jct_y);
                            min_dst_id = seg2_r->getNodeId1();
                            min_dst_pt = seg2_r->getP2();
                        }
                    }
                    if (min_dst < 0) return;

                    std::unique_ptr<Resistor> seg2_r_escape(new Resistor(escape_jct_id,
                        min_dst_id,
                        seg2->getLayerName(),
                        min_dst,
                        seg2_rect.ur.x - seg1_rect.ll.x,
                        -1,
                        Point2D<double>(split_pt.x, escape_jct_y),
                        min_dst_pt,
                        seg2
                    ));
                    _resistor_network.emplace_back(std::move(seg2_r_escape));
                    return;
                }
            }

        } else if (seg2_p1.y == seg2_p2.y && seg2_p1.y > std::min(seg1_p1.y, seg1_p2.y) && seg2_p1.y < std::max(seg1_p1.y, seg1_p2.y)) { // seg 1 vertical, cuts seg 2
            for (Resistor *res : seg1->getResistors()) {
                Point2D<double> res_p1 = res->getP1();
                Point2D<double> res_p2 = res->getP2();

                if (seg2_p1.y > std::min(res_p1.y, res_p2.y) && seg2_p1.y < std::max(res_p1.y, res_p2.y)) {
                    Point2D<double> split_pt(res_p1.x, seg1_p1.y);
                    std::string new_id = _splitResistorAtPt(res, split_pt);
                    double escape_jct_x = seg2_p1.x > res_p1.x ? seg1_rect.ur.x : seg1_rect.ll.x;
                    std::string escape_jct_id = generateNodeID(seg1->getNetName());

                    std::unique_ptr<Resistor> seg1_r_escape(new Resistor(new_id,
                        escape_jct_id,
                        seg1->getLayerName(),
                        abs(split_pt.x - escape_jct_x),
                        seg2_rect.ur.y - seg2_rect.ll.y,
                        -1,
                        split_pt,
                        Point2D<double>(escape_jct_x, split_pt.y),
                        seg1
                    ));
                    _resistor_network.emplace_back(std::move(seg1_r_escape));

                    double min_dst = -1;
                    std::string  min_dst_id = "";
                    Point2D<double> min_dst_pt;
                    for (Resistor *seg2_r : seg2->getResistors()) {
                        if (min_dst < 0 || abs(seg2_r->getP1().x - escape_jct_x) < min_dst) {
                            min_dst = abs(seg2_r->getP1().x - escape_jct_x);
                            min_dst_id = seg2_r->getNodeId1();
                            min_dst_pt = seg2_r->getP1();
                        }

                        if (min_dst < 0 || abs(seg2_r->getP2().x - escape_jct_x) < min_dst) {
                            min_dst = abs(seg2_r->getP2().x - escape_jct_x);
                            min_dst_id = seg2_r->getNodeId1();
                            min_dst_pt = seg2_r->getP2();
                        }
                    }
                    if (min_dst < 0) return;

                    std::unique_ptr<Resistor> seg2_r_escape(new Resistor(escape_jct_id,
                        min_dst_id,
                        seg2->getLayerName(),
                        min_dst,
                        seg2_rect.ur.y - seg2_rect.ll.y,
                        -1,
                        Point2D<double>(escape_jct_x, split_pt.y),
                        min_dst_pt,
                        seg2
                    ));
                    _resistor_network.emplace_back(std::move(seg2_r_escape));
                    return;
                }
            }
        }

    }

    void Geometry::_handleContains(WireSegment *super_seg, WireSegment *sub_seg) {
        if (sub_seg->getResistors().size() == 0) return; // if sub segment has no resistors then no work to be done!

        Rect2D<double> super_seg_rect = super_seg->getRect();

        bool sub_seg_has_vertical_pred = sub_seg->getVerticalConnections().size() > 0;
        Point2D<double> sub_seg_pt = sub_seg_has_vertical_pred ? sub_seg->getP2() : sub_seg->getP1(); // just in case they're different be aware of existing resistance structure

        if (super_seg->getResistors().size() == 0) {
            // no resistor for seg, need to generate one

            Point2D<double> center((super_seg_rect.ll.x + super_seg_rect.ur.x) / 2, (super_seg_rect.ll.y + super_seg_rect.ur.y) / 2);
            double area = (super_seg_rect.ur.x - super_seg_rect.ll.x) * (super_seg_rect.ur.y - super_seg_rect.ll.y);

            if (sub_seg_has_vertical_pred) { // has a vertical pred ==> has resistor below it so connect to its top
                Resistor *sub_seg_resistor = sub_seg->getResistors()[0];

                std::unique_ptr<Resistor> resistor(new Resistor(sub_seg_resistor->getNodeId2(),
                    generateNodeID(super_seg->getNetName()),
                    super_seg->getLayerName(),
                    -1,
                    -1,
                    area,
                    sub_seg_pt,
                    center,
                    super_seg
                ));
                _resistor_network.emplace_back(std::move(resistor));
            } else { // has resistor above it, connect to its bottom
                Resistor *sub_seg_resistor = sub_seg->getResistors()[0];

                std::unique_ptr<Resistor> resistor(new Resistor(generateNodeID(super_seg->getNetName()),
                    sub_seg_resistor->getNodeId1(),
                    super_seg->getLayerName(),
                    -1,
                    -1,
                    area,
                    center,
                    sub_seg_pt,
                    super_seg
                ));
                _resistor_network.emplace_back(std::move(resistor));
            }
        } else {
            for (Resistor *r : super_seg->getResistors()) {
                Point2D<double> res_p1 = r->getP1();
                Point2D<double> res_p2 = r->getP2();
                if (res_p1.x == sub_seg_pt.x && res_p1.y == sub_seg_pt.y) { // length wise resistor shares point with sub segment
                    if (sub_seg_has_vertical_pred) {
                        sub_seg->getResistors()[0]->setNodeId2(r->getNodeId1());
                    } else {
                        if ((r->getArea() == -1 ? r->getNodeId1() : r->getNodeId2()) == sub_seg->getResistors()[0]->getNodeId2()) {
                            std::cout << "START BAD BLAH BLAH" << std::endl;
                            r->print(std::cout);
                            sub_seg->getResistors()[0]->print(std::cout);
                            std::cout << "END BAD BLAH BLAH" << std::endl;
                        }
                        sub_seg->getResistors()[0]->setNodeId1(r->getArea() == -1 ? r->getNodeId1() : r->getNodeId2());
                    }
                    return;
                } else if (res_p2.x == sub_seg_pt.x && res_p2.y == sub_seg_pt.y) {
                    if (sub_seg_has_vertical_pred) {
                        sub_seg->getResistors()[0]->setNodeId2(r->getArea() == -1 ? r->getNodeId2() : r->getNodeId1());
                    } else {
                        sub_seg->getResistors()[0]->setNodeId1(r->getNodeId2());
                    }
                    return;
                } else if (r->getArea() == -1 && res_p2.x == res_p1.x && res_p2.x == sub_seg_pt.x && sub_seg_pt.y > std::min(res_p1.y, res_p2.y) && sub_seg_pt.y < std::max(res_p1.y, res_p2.y)) { // other_seg falls in middle of res
                    std::string new_node_id = _splitResistorAtPt(r, sub_seg_pt);
                    if (sub_seg_has_vertical_pred) {
                        sub_seg->getResistors()[0]->setNodeId2(new_node_id);
                    } else {
                        sub_seg->getResistors()[0]->setNodeId1(new_node_id);
                    }
                    return;
                } else if (r->getArea() == -1 && res_p2.y == res_p1.y && res_p2.y == sub_seg_pt.y && sub_seg_pt.x > std::min(res_p1.x, res_p2.x) && sub_seg_pt.x < std::max(res_p1.x, res_p2.x)) { // other_seg falls in middle of res
                    std::string new_node_id = _splitResistorAtPt(r, sub_seg_pt);
                    if (sub_seg_has_vertical_pred) {
                        sub_seg->getResistors()[0]->setNodeId2(new_node_id);
                    } else {
                        sub_seg->getResistors()[0]->setNodeId1(new_node_id);
                    }
                    return;
                }
            }
        }
    }
    

     std::vector<std::unique_ptr<WireSegment>> &Geometry::getSegmentsOfNet(std::string net) {
        if (_net_to_segs.find(net) == _net_to_segs.end()) {
            _net_to_segs.emplace(net, std::vector<std::unique_ptr<WireSegment>>());
        }
        return _net_to_segs.at(net);
    }

    WireSegment *Geometry::addSegmentToNet(std::string net, WireSegment &seg) {
        std::unique_ptr<WireSegment> seg_ptr(new WireSegment(seg));
        if (_net_to_segs.find(net) == _net_to_segs.end()) {
            _net_to_segs.emplace(net, std::vector<std::unique_ptr<WireSegment>>());
        }
        _net_to_segs.at(net).push_back(std::move(seg_ptr));
        return _net_to_segs.at(net).back().get();
    }

    WireSegment *Geometry::addSegmentToLayer(std::string layer, WireSegment *seg_ptr) {
        
        if (_layer_to_partitioned_segs.find(layer) == _layer_to_partitioned_segs.end()) {
            _layer_to_partitioned_segs.emplace(layer, UniformPartition<WireSegment>(_default_partition_size));
        }

        _layer_to_partitioned_segs.at(layer).add(seg_ptr);
        return seg_ptr;
    }

    void Geometry::_populateCapacitanceNetwork() {
        if (_capacitor_network.size() > 0) {
            std::cout << "something odd! capacitor network already populated...";
            return;
        }

        for (std::map<std::string, std::vector<std::unique_ptr<WireSegment>>>::iterator it = _net_to_segs.begin() ; it != _net_to_segs.end() ; it++) {
            std::string net = it->first;
            for (std::unique_ptr<WireSegment> &seg : it->second) {
                if (seg->getResistors().size() == 0) continue;
                std::vector<WireSegment *> nbr_ptrs = getOtherNetsNearbySegments(seg.get());
                for (WireSegment *nbr_ptr : nbr_ptrs) {
                    if (nbr_ptr->getResistors().size() == 0) continue;
                    Rect2D<double> seg_rect = seg->getRect();
                    Rect2D<double> nbr_rect = nbr_ptr->getRect();
                    if (seg_rect.ll.y < nbr_rect.ll.y) { // double counting problem
                        continue;
                    }
                    
                    double overlap_length = 0.0, distance = 0.0, overlap_center = 0.0;


                    // overlap in the x direction
                    if ((seg_rect.ll.x <= nbr_rect.ur.x && seg_rect.ur.x >= nbr_rect.ll.x) || (nbr_rect.ll.x <= seg_rect.ur.x && nbr_rect.ur.x >= seg_rect.ll.x)) {
                        overlap_length = std::min(seg_rect.ur.x, nbr_rect.ur.x) - std::max(seg_rect.ll.x, nbr_rect.ll.x);
                        distance = std::min(std::abs(nbr_rect.ll.y - seg_rect.ur.y), std::abs(nbr_rect.ur.y - seg_rect.ll.y));

                        overlap_center = (std::min(seg_rect.ur.x, nbr_rect.ur.x) + std::max(seg_rect.ll.x, nbr_rect.ll.x)) / 2;

                        std::string seg_split_id;
                        for (Resistor *seg_res : seg->getResistors()) {
                            Point2D<double> seg_res_p1 = seg_res->getP1();
                            Point2D<double> seg_res_p2 = seg_res->getP2();

                            if (seg_res_p1.y == seg_res_p2.y && overlap_center > std::min(seg_res_p1.x, seg_res_p2.x) && overlap_center < std::max(seg_res_p1.x, seg_res_p2.x)) {
                                seg_split_id = _splitResistorAtPt(seg_res, Point2D<double>(overlap_center, seg_res_p1.y));
                                break;
                            }
                        }

                        if (seg_split_id.length() == 0) {
                            double overlap_left = std::max(seg_rect.ll.x, nbr_rect.ll.x);
                            double overlap_right = std::min(seg_rect.ur.x, nbr_rect.ur.x);
                            for (Resistor *seg_res : seg->getResistors()) {
                                Point2D<double> seg_res_p1 = seg_res->getP1();
                                Point2D<double> seg_res_p2 = seg_res->getP2();

                                if (seg_res_p1.y == seg_res_p2.y &&
                                    ((overlap_right > std::min(seg_res_p1.x, seg_res_p2.x) && overlap_right < std::max(seg_res_p1.x, seg_res_p2.x)) ||
                                    (overlap_left > std::min(seg_res_p1.x, seg_res_p2.x) && overlap_left < std::max(seg_res_p1.x, seg_res_p2.x)))) {
                                    
                                    double sub_overlap_center = (std::max(overlap_left, std::min(seg_res_p1.x, seg_res_p2.x)) + std::min(overlap_right, std::max(seg_res_p1.x, seg_res_p2.x))) /2;

                                    seg_split_id = _splitResistorAtPt(seg_res, Point2D<double>(sub_overlap_center, seg_res_p1.y));
                                    break;
                                }
                            }
                        }

                        std::string nbr_split_id;
                        for (Resistor *nbr_res : seg->getResistors()) {
                            Point2D<double> nbr_res_p1 = nbr_res->getP1();
                            Point2D<double> nbr_res_p2 = nbr_res->getP2();

                            if (nbr_res_p1.y == nbr_res_p2.y && overlap_center > std::min(nbr_res_p1.x, nbr_res_p2.x) && overlap_center < std::max(nbr_res_p1.x, nbr_res_p2.x)) {
                                nbr_split_id = _splitResistorAtPt(nbr_res, Point2D<double>(overlap_center, nbr_res_p1.y));
                                break;
                            }
                        }

                        if (nbr_split_id.length() == 0) {
                            double overlap_left = std::max(nbr_rect.ll.x, seg_rect.ll.x);
                            double overlap_right = std::min(nbr_rect.ur.x, seg_rect.ur.x);
                            for (Resistor *nbr_res : nbr_ptr->getResistors()) {
                                Point2D<double> nbr_res_p1 = nbr_res->getP1();
                                Point2D<double> nbr_res_p2 = nbr_res->getP2();

                                if (nbr_res_p1.y == nbr_res_p2.y &&
                                    ((overlap_right > std::min(nbr_res_p1.x, nbr_res_p2.x) && overlap_right < std::max(nbr_res_p1.x, nbr_res_p2.x)) ||
                                    (overlap_left > std::min(nbr_res_p1.x, nbr_res_p2.x) && overlap_left < std::max(nbr_res_p1.x, nbr_res_p2.x)))) {
                                    
                                    double sub_overlap_center = (std::max(overlap_left, std::min(nbr_res_p1.x, nbr_res_p2.x)) + std::min(overlap_right, std::max(nbr_res_p1.x, nbr_res_p2.x))) /2;

                                    nbr_split_id = _splitResistorAtPt(nbr_res, Point2D<double>(sub_overlap_center, nbr_res_p1.y));
                                    break;
                                }
                            }
                        }

                        if (!seg_split_id.empty() && !nbr_split_id.empty()) {
                            std::unique_ptr<Capacitor> capacitor(new Capacitor(
                                seg_split_id,
                                nbr_split_id,
                                seg->getLayerName(),
                                overlap_length,
                                distance
                            ));
                            _capacitor_network.emplace_back(std::move(capacitor));
                        }
                    }
                    // overlap in the y direction
                    else if ((seg_rect.ll.y <= nbr_rect.ur.y && seg_rect.ur.y >= nbr_rect.ll.y) || (nbr_rect.ll.y <= seg_rect.ur.y && nbr_rect.ur.y >= seg_rect.ll.y)) {
                        overlap_length = std::min(seg_rect.ur.y, nbr_rect.ur.y) - std::max(seg_rect.ll.y, nbr_rect.ll.y);
                        distance = std::min(std::abs(nbr_rect.ll.x - seg_rect.ur.x), std::abs(nbr_rect.ur.x - seg_rect.ll.x));

                        overlap_center = (std::min(seg_rect.ur.y, nbr_rect.ur.y) + std::max(seg_rect.ll.y, nbr_rect.ll.y)) / 2;

                        std::string seg_split_id;
                        for (Resistor *seg_res : seg->getResistors()) {
                            Point2D<double> seg_res_p1 = seg_res->getP1();
                            Point2D<double> seg_res_p2 = seg_res->getP2();

                            if (seg_res_p1.x == seg_res_p2.x && overlap_center > std::min(seg_res_p1.y, seg_res_p2.y) && overlap_center < std::max(seg_res_p1.y, seg_res_p2.y)) {
                                seg_split_id = _splitResistorAtPt(seg_res, Point2D<double>(seg_res_p1.x, overlap_center));
                                break;
                            }
                        }

                        if (seg_split_id.length() == 0) {
                            double overlap_left = std::max(seg_rect.ll.y, nbr_rect.ll.y);
                            double overlap_right = std::min(seg_rect.ur.y, nbr_rect.ur.y);
                            for (Resistor *seg_res : seg->getResistors()) {
                                Point2D<double> seg_res_p1 = seg_res->getP1();
                                Point2D<double> seg_res_p2 = seg_res->getP2();

                                if (seg_res_p1.x == seg_res_p2.x &&
                                    ((overlap_right > std::min(seg_res_p1.y, seg_res_p2.y) && overlap_right < std::max(seg_res_p1.y, seg_res_p2.y)) ||
                                    (overlap_left > std::min(seg_res_p1.y, seg_res_p2.y) && overlap_left < std::max(seg_res_p1.y, seg_res_p2.y)))) {
                                    
                                    double sub_overlap_center = (std::max(overlap_left, std::min(seg_res_p1.y, seg_res_p2.y)) + std::min(overlap_right, std::max(seg_res_p1.y, seg_res_p2.y))) /2;

                                    seg_split_id = _splitResistorAtPt(seg_res, Point2D<double>(seg_res_p1.x, sub_overlap_center));
                                    break;
                                }
                            }
                        }

                        std::string nbr_split_id;
                        for (Resistor *nbr_res : seg->getResistors()) {
                            Point2D<double> nbr_res_p1 = nbr_res->getP1();
                            Point2D<double> nbr_res_p2 = nbr_res->getP2();

                            if (nbr_res_p1.x == nbr_res_p2.x && overlap_center > std::min(nbr_res_p1.y, nbr_res_p2.y) && overlap_center < std::max(nbr_res_p1.y, nbr_res_p2.y)) {
                                seg_split_id = _splitResistorAtPt(nbr_res, Point2D<double>(nbr_res_p1.x, overlap_center));
                                break;
                            }
                        }

                        if (nbr_split_id.length() == 0) {
                            double overlap_left = std::max(nbr_rect.ll.y, seg_rect.ll.y);
                            double overlap_right = std::min(nbr_rect.ur.y, seg_rect.ur.y);
                            for (Resistor *nbr_res : nbr_ptr->getResistors()) {
                                Point2D<double> nbr_res_p1 = nbr_res->getP1();
                                Point2D<double> nbr_res_p2 = nbr_res->getP2();

                                if (nbr_res_p1.x == nbr_res_p2.x &&
                                    ((overlap_right > std::min(nbr_res_p1.y, nbr_res_p2.y) && overlap_right < std::max(nbr_res_p1.y, nbr_res_p2.y)) ||
                                    (overlap_left > std::min(nbr_res_p1.y, nbr_res_p2.y) && overlap_left < std::max(nbr_res_p1.y, nbr_res_p2.y)))) {
                                    
                                    double sub_overlap_center = (std::max(overlap_left, std::min(nbr_res_p1.y, nbr_res_p2.y)) + std::min(overlap_right, std::max(nbr_res_p1.y, nbr_res_p2.y))) /2;

                                    nbr_split_id = _splitResistorAtPt(nbr_res, Point2D<double>(sub_overlap_center, nbr_res_p1.x));
                                    break;
                                }
                            }
                        }

                        if (!seg_split_id.empty() && !nbr_split_id.empty()) { // not enough overlap otherwise given model

                            std::unique_ptr<Capacitor> capacitor(new Capacitor(
                                seg_split_id,
                                nbr_split_id,
                                seg->getLayerName(),
                                overlap_length,
                                distance
                            ));
                            _capacitor_network.emplace_back(std::move(capacitor));
                        }
                    } else {
                        // std::cout << "NON OVERLAPPING NBR: " << nbr_rect << std::endl;
                        continue;
                    }

                    // std::cout << "SEGMENT #" << seg->getSegmentNumber() << " OF NET '" << seg->getNetName() << "' HAS OVERLAPPING REGION OF LENGTH " << overlap_length << " AT DISTANCE " << distance << " WITH SEGMENT #" << nbr_ptr->getSegmentNumber() << " OF NET '" << nbr_ptr->getNetName() << "'" << std::endl; 
                }
            }
        }
    }

    std::vector<WireSegment *> Geometry::getOtherNetsNearbySegments(WireSegment *seg_ptr) {
        if (_layer_to_partitioned_segs.find(seg_ptr->getLayerName()) == _layer_to_partitioned_segs.end()) {
            std::cout << "PROBLEM PROBLEM WITH LAYER (" << seg_ptr->getLayerName() << ")" << std::endl;
            exit(1);
        }

        UniformPartition<WireSegment> &layer_segs = _layer_to_partitioned_segs.at(seg_ptr->getLayerName());
        std::pair<int, int> ll_partition = layer_segs.getPartitionId(seg_ptr->getRect().ll);
        std::pair<int, int> ur_partition = layer_segs.getPartitionId(seg_ptr->getRect().ur);

        std::set<WireSegment *> nbr_ptr_set;

        for (int x_partition = ll_partition.first - _num_bins_neighborhood; x_partition <= ur_partition.first + _num_bins_neighborhood; x_partition++) {
            for (int y_partition = ll_partition.second - _num_bins_neighborhood; y_partition <= ur_partition.second + _num_bins_neighborhood; y_partition++) {
                std::pair<int, int> curr_partition(x_partition, y_partition);
                std::vector<WireSegment *> segments_of_partition = layer_segs.getElementsByPartition(curr_partition);
                for (WireSegment *other_seg_ptr : segments_of_partition) {
                    if (other_seg_ptr->getNetName() != seg_ptr->getNetName()) {
                        nbr_ptr_set.insert(other_seg_ptr);
                    }
                }
            }
        }
        return std::vector<WireSegment *>(nbr_ptr_set.begin(), nbr_ptr_set.end()); // use set iterator to construct vector
    }

    void Geometry::printRCNetwork(std::ostream &stream) {
        for (std::unique_ptr<Resistor> &res : _resistor_network) {
            res->print(stream);
        }

        for (std::unique_ptr<Capacitor> &cap : _capacitor_network) {
            cap->print(stream);
        }
    }

    template<typename T>
    std::vector<T *> UniformPartition<T>::getElementsByPartition(std::pair<int, int> partition) const {
        if (_partition_pair_to_segs.find(partition) == _partition_pair_to_segs.end()) {
            return std::vector<T *>();
        }
        return _partition_pair_to_segs.at(partition);
    }

    template<typename T>
    void UniformPartition<T>::add(T *elt) {
        Rect2D<double> elt_rect = elt->getRect();

        int left_bound_idx = static_cast<int>(elt_rect.ll.x / _partition_size);
        int right_bound_idx = static_cast<int>(elt_rect.ur.x/ _partition_size);

        int bottom_bound_idx = static_cast<int>(elt_rect.ll.y / _partition_size);
        int top_bound_idx = static_cast<int>(elt_rect.ur.y / _partition_size);

        for (int horiz_partition = left_bound_idx; horiz_partition <= right_bound_idx; horiz_partition++) {
            for (int vert_partition = bottom_bound_idx ; vert_partition <= top_bound_idx; vert_partition++) {
                std::pair<int, int> partition_pair = { horiz_partition, vert_partition };
                if (_partition_pair_to_segs.find(partition_pair) == _partition_pair_to_segs.end()) {
                    _partition_pair_to_segs[partition_pair] = std::vector<T *>();
                }
                _partition_pair_to_segs.at(partition_pair).push_back(elt);
            }
        }
    }
}