////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//            Copyright© 2026 Solex Robot, All Rights Reserved.               //
//                                                                            //
//  All users are hereby notified that the materials in the form of digital   //
//  information available from this software (content, designs, color         //
//  schemes, graphic styles, images, logo, text, and videos) comes protected  //
//  under International Copyright Laws. Therefore it should not be reproduced //
//  in any form digital or offline without prior written permission of        //
//  Solex Robot.                                                              //
//                                                                            //
//  Any unauthorized reprint or material usage (Solex Robot) either manually  //
//  or digitally, is strictly prohibited.                                     //
//                                                                            //
//  Any further unauthorized digital copying of this material via copying,    //
//  publication, reproduction or distribution of copyrighted works is an      //
//  infringement of the copyright owners' rights may be the subject of the    //
//  copyright of performers' protection under the Copyright Act. For such     //
//  illegal activities you will be strictly liable to Solox Robot for any and //
//  or all damages (including recovery of attorneys' fees) which may be       //
//  suffered and or incurred as a result of your infringement.                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cmath>
#include <cstring>
#include <queue>

namespace solex_robot::navigation::localization_2d {
namespace {
constexpr float kMaxOccupiedDistance = 3.0;
}

struct CellIndex {
  int x = 0;
  int y = 0;
};

struct CellData {
  CellIndex cell_index;
  CellIndex obstacle_index;
  float min_obstacle_distance = 0.0;
};
using CellDataPtr = std::shared_ptr<CellData>;

std::vector<float> ComputeDistanceField(
    const std::vector<int8_t>& occupied_cells, const int height,
    const int width) {
  std::queue<CellDataPtr> cell_queue;
  std::vector<bool> mask(occupied_cells.size(), false);

  std::vector<CellDataPtr> cell_datas;
  cell_datas.reserve(occupied_cells.size());
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;

      CellDataPtr cell_data = std::make_shared<CellData>();
      cell_data->cell_index.x = x;
      cell_data->cell_index.y = y;
      cell_data->min_obstacle_distance = kMaxOccupiedDistance;
      if (occupied_cells[index] == 100) {
        cell_data->obstacle_index.x = x;
        cell_data->obstacle_index.y = y;
        cell_data->min_obstacle_distance = 0.0;
        cell_queue.emplace(cell_data);
        mask[index] = 1;
      }
      cell_datas.emplace_back(cell_data);
    }
  }

  auto enqueue = [&](const CellIndex& cell_index,
                     const CellIndex& obstacle_index) {
    const int index = cell_index.y * width + cell_index.x;
    if (mask[index]) {
      return;
    }

    const float distance =
        std::sqrt(std::pow(cell_index.x - obstacle_index.x, 2) +
                  std::pow(cell_index.y - obstacle_index.y, 2));
    if (distance > kMaxOccupiedDistance) {
      return;
    }

    cell_datas[index]->min_obstacle_distance = distance;
    cell_datas[index]->obstacle_index = obstacle_index;
    
    cell_queue.emplace(cell_datas[index]);
    mask[index] = 1;
  };

  while (!cell_queue.empty()) {
    const CellDataPtr current_cell = cell_queue.front();
    cell_queue.pop();
    if (current_cell->cell_index.x > 0) {
      CellIndex next_cell_index = current_cell->cell_index;
      next_cell_index.x -= 1;
      enqueue(next_cell_index, current_cell->obstacle_index);
    }

    if (current_cell->cell_index.y > 0) {
      CellIndex next_cell_index = current_cell->cell_index;
      next_cell_index.y -= 1;
      enqueue(next_cell_index, current_cell->obstacle_index);
    }

    if (current_cell->cell_index.x + 1 < width) {
      CellIndex next_cell_index = current_cell->cell_index;
      next_cell_index.x += 1;
      enqueue(next_cell_index, current_cell->obstacle_index);
    }

    if (current_cell->cell_index.y + 1 < height) {
      CellIndex next_cell_index = current_cell->cell_index;
      next_cell_index.y += 1;
      enqueue(next_cell_index, current_cell->obstacle_index);
    }
  }

  std::cout << "cell_datas = " << cell_datas.size() << std::endl;
  std::cout << "occupied_cells = " << occupied_cells.size() << std::endl;

  std::vector<float> distance_field;
  distance_field.reserve(cell_datas.size());
  for (const auto& cell_data : cell_datas) {
    distance_field.emplace_back(cell_data->min_obstacle_distance);
  }

  return distance_field;
}

}  // namespace solex_robot::navigation::localization_2d