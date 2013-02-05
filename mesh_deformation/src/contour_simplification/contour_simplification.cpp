//
//  contour_simplification.cpp
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//

#include <fstream>
#include <iostream>
#include <string>
#include "contour_simplification/contour_simplification.h"
#include "contour_simplification/min_heap_contours.h"
#include "exceptions/wruntime_error.h"

using data_str::Vector;
using math::Float3;
using math::Float2;
using std::wruntime_error;
using std::wstring;

namespace contour_simplification {

  void ContourSimplification::simplifyContour(uint32_t target_contour_count,
      Vector<Float3>* vertices, Vector<unsigned char>* mask,
      uint32_t width, uint32_t height) {
    width_ = width;
    height_ = height;

    marchingSquares(vertices, mask);

    createMinHeap();

    cullContours(target_contour_count, vertices);
  }

  void ContourSimplification::marchingSquares(Vector<Float3>* vertices, 
    Vector<unsigned char>* mask) {
    // NOTE: We're going to add a zero boarder around the coded image so that
    // despite contours intersecting the boundry they always form a complete
    // contour
    if (coded_image_.capacity() < (width_ + 2)*(height_ + 2)) {
      coded_image_.capacity((width_ + 2)*(height_ + 2));
    }
    coded_image_.resize((width_ + 2)*(height_ + 2));

    // Zero the boarder - top and bottom
    for (uint32_t v = 0; v < (height_ + 2); v += height_ + 1) {
      for (uint32_t u = 0; u < (width_ + 2); u++) {
        coded_image_[v*(width_+2) + u] = 0;
      }
    }
    // Zero the boarder - left and right
    for (uint32_t u = 0; u < (width_ + 2); u += width_ + 1) {
      for (uint32_t v = 1; v < (height_ + 1); v++) {
        coded_image_[v*(width_+2) + u] = 0;
      }
    }

    contours_.resize(0);
    contours_starts_.resize(0);

    // Step through the image, creating a coded representation of the
    // contours.
    uint16_t verts_case;
    uint32_t p0, p1, p2, p3;
    for (uint32_t v = 0; v < height_-1; v++) {
      for (uint32_t u = 0; u < width_-1; u++) {
        p0 = v*width_ + u;          // top left
        p1 = v*width_ + (u+1);      // top right
        p2 = (v+1)*width_ + u;      // bottom left
        p3 = (v+1)*width_ + (u+1);  // bottom right
        // For each box of 4 vertices, if 3 of them have valid points, add
        // a triangle, if 4 of them have vertices add 4 triangles
        verts_case = 0;
        if ((*mask)[p0]) {
          verts_case = verts_case | 1;
        }
        if ((*mask)[p1]) {
          verts_case = verts_case | 2;
        }
        if ((*mask)[p2]) {
          verts_case = verts_case | 4;
        }
        if ((*mask)[p3]) {
          verts_case = verts_case | 8;
        }
        uint32_t coded_image_index = (v + 1) * (width_ + 2) + (u + 1);
        coded_image_[coded_image_index] = verts_case;
      }  // for (uint32_t u = 0; u < width; u++)
    }  // for (uint32_t v = 0; v < height; v++)

    // Step through the image, walking around the countours when we see them.
    // For consistency, we ALWAYS walk around a contour keeping the "1" value
    // to our left.
    Float3 v1;
    Float3 v2;
    uint32_t num_contours_ = 0;
    for (uint32_t v = 0; v < height_ + 1; v++) {
      for (uint32_t u = 0; u < width_ + 1; u++) {
        uint32_t start_coded_image = v*(width_ + 2) + u;
        uint32_t cur_coded_image = start_coded_image;
        uint32_t contour_start = contours_.size();
        MS_DIRECTION direction_prev_point = MS_UNDEFINED;  // Where we came from
        do {
          uint16_t cur_code = coded_image_[cur_coded_image];
          coded_image_[cur_coded_image] = 16;  // Indicates it's already been visited

          // Early out for the vast majority of points:
          if (cur_code == 0 || cur_code == 15 || cur_code == 16) {
            // Finish the contour if we had one
            break;
          }

          // Note: Vertex indices are off by 1
          uint32_t u_coded_image = cur_coded_image % (width_ + 2);
          uint32_t v_coded_image = cur_coded_image / (width_ + 2);
          p0 = (v_coded_image - 1) * width_ + (u_coded_image - 1);  // top left
          p1 = p0 + 1;                                              // top right
          p2 = p0 + width_;                                         // bottom left
          p3 = p0 + width_ + 1;                                     // bottom right

          // There are only a few cases that we care about.  
          switch (cur_code) {
          case 1:  // Top left
            appendContour(vertices->at(p0), contour_start, num_contours_);
            stepUp(&cur_coded_image, &direction_prev_point);
            break;
          case 2:  // Top right
            appendContour(vertices->at(p1), contour_start, num_contours_);
            stepRight(&cur_coded_image, &direction_prev_point);
            break;
          case 3:  // Top left and Top Right
            appendContour(vertices->at(p0), contour_start, num_contours_);
            appendContour(vertices->at(p1), contour_start, num_contours_);
            stepRight(&cur_coded_image, &direction_prev_point);
            break;
          case 4:  // Bottom Left
            appendContour(vertices->at(p2), contour_start, num_contours_);
            stepLeft(&cur_coded_image, &direction_prev_point);
            break;
          case 5:  // Top left and Bottom Left
            appendContour(vertices->at(p2), contour_start, num_contours_);
            appendContour(vertices->at(p0), contour_start, num_contours_);
            stepUp(&cur_coded_image, &direction_prev_point);
            break;
          case 6:  // Top Right and Bottom Left **** SADDLE POINT ****
            if (direction_prev_point == MS_UP) {
              appendContour(vertices->at(p1), contour_start, num_contours_);
              appendContour(vertices->at(p2), contour_start, num_contours_);
              coded_image_[cur_coded_image] = 6;
              stepLeft(&cur_coded_image, &direction_prev_point);
            } else if (direction_prev_point == MS_DOWN) {
              appendContour(vertices->at(p2), contour_start, num_contours_);
              appendContour(vertices->at(p1), contour_start, num_contours_);
              coded_image_[cur_coded_image] = 6;
              stepRight(&cur_coded_image, &direction_prev_point);
            }
            break;
          case 7:  // Top Left, Top Right, Bottom Left
            appendContour(vertices->at(p2), contour_start, num_contours_);
            appendContour(vertices->at(p1), contour_start, num_contours_);
            stepRight(&cur_coded_image, &direction_prev_point);
            break;
          case 8:  // Bottom right
            appendContour(vertices->at(p3), contour_start, num_contours_);
            stepDown(&cur_coded_image, &direction_prev_point);
            break;
          case 9:  // Top Right and Bottom Left **** SADDLE POINT ****
            // I arbitrarily choose to make hour-glass (saddle points) 
            // connected, so there is one continuous contour
            if (direction_prev_point == MS_RIGHT) {
              appendContour(vertices->at(p3), contour_start, num_contours_);
              appendContour(vertices->at(p1), contour_start, num_contours_);
              coded_image_[cur_coded_image] = 9;
              stepUp(&cur_coded_image, &direction_prev_point);
            } else if (direction_prev_point == MS_LEFT) {
              appendContour(vertices->at(p0), contour_start, num_contours_);
              appendContour(vertices->at(p3), contour_start, num_contours_);
              coded_image_[cur_coded_image] = 9;
              stepDown(&cur_coded_image, &direction_prev_point);
            }
            break;
          case 10:  // Bottom right and Top right
            appendContour(vertices->at(p1), contour_start, num_contours_);
            appendContour(vertices->at(p3), contour_start, num_contours_);
            stepDown(&cur_coded_image, &direction_prev_point);
            break;
          case 11:  // Bottom right, top left, top right
            appendContour(vertices->at(p0), contour_start, num_contours_);
            appendContour(vertices->at(p3), contour_start, num_contours_);
            stepDown(&cur_coded_image, &direction_prev_point);
            break;
          case 12:  // Bottom right, bottom left
            appendContour(vertices->at(p3), contour_start, num_contours_);
            appendContour(vertices->at(p2), contour_start, num_contours_);
            stepLeft(&cur_coded_image, &direction_prev_point);
            break;
          case 13:  // Bottom right, bottom left, top left
            appendContour(vertices->at(p3), contour_start, num_contours_);
            appendContour(vertices->at(p0), contour_start, num_contours_);
            stepUp(&cur_coded_image, &direction_prev_point);
            break;
          case 14:  // Bottom right, bottom left, top right
            appendContour(vertices->at(p1), contour_start, num_contours_);
            appendContour(vertices->at(p2), contour_start, num_contours_);
            stepLeft(&cur_coded_image, &direction_prev_point);
            break;
          }
        } while (cur_coded_image != start_coded_image);
        if (direction_prev_point != MS_UNDEFINED) {
          finishLastContour(contour_start);
          num_contours_++;
          contours_starts_.pushBack(contour_start);
        }
      }  // for (uint32_t u = 0; u < width; u++)
    }  // for (uint32_t v = 0; v < height; v++)

    // Cull redundant contours (of size 0)
    for (uint32_t i = 0; i < contours_starts_.size(); i++) {
      uint32_t start_contour = contours_starts_[i];
      uint32_t cur_contour = start_contour;
      bool finished = false;
      do {
        Contour* cont = contours_.at(cur_contour);
        Contour* cont_next = contours_.at(cont->next);
        Contour* cont_next_next = contours_.at(cont_next->next);
        while (cont != cont_next &&
               Float3::equal(&cont->v1, &cont_next->v1)) {
          // Skip over the next contour since it's position is the same as mine
          cont->next = cont_next_next->curr;
          cont_next_next->prev = cont->curr;
          // Also invalidate the contour

          // Are we deleting the start index on the second time through?
          if (cont_next->curr == start_contour) {
            finished = true;
          }
          cont_next->invalidateContour();
          
          cont_next = cont_next_next;
          cont_next_next = contours_.at(cont_next->next);
        }
        cur_contour = cont->next;
      } while (cur_contour != start_contour && !finished);
    }
    
    fixContourStarts();
  }

  void ContourSimplification::appendContour(Float3* vertex, 
    uint32_t contour_start, uint32_t contour_index) {
    uint32_t index = contours_.size();
    contours_.pushBack(Contour(vertex, contour_index, index));
    if (contours_.size() - 1 != contour_start) {  
      // Only join the contours if we aren't starting a new one
      contours_.at(contours_.size()-2)->next = contours_.size()-1;
      contours_.at(contours_.size()-1)->prev = contours_.size()-2;
    }
  }

  void ContourSimplification::finishLastContour(uint32_t contour_start) {
     if (contours_.size() > 1) {
       contours_.at(contours_.size() - 1)->next = contour_start;
       contours_.at(contour_start)->prev = contours_.size() - 1;
     }
  }

  void ContourSimplification::stepUp(uint32_t* point, 
    MS_DIRECTION* prev_direction) {
    if ((*point / (width_ + 2)) > 0) {
      *point = *point - (width_ + 2);
      *prev_direction = MS_DOWN;
    }
  }

  void ContourSimplification::stepDown(uint32_t* point, 
    MS_DIRECTION* prev_direction) {
    if ((*point / (width_ + 2)) < (height_ + 1)) {
      *point = *point + (width_ + 2);
      *prev_direction = MS_UP;
    }
  }

  void ContourSimplification::stepLeft(uint32_t* point, 
    MS_DIRECTION* prev_direction) {
    if ((*point % (width_ + 2)) > 0) {
      *point = *point - 1;
      *prev_direction = MS_RIGHT;
    }
  }

  void ContourSimplification::stepRight(uint32_t* point, 
    MS_DIRECTION* prev_direction) {
    if ((*point % (width_ + 2)) < (width_ + 1)) {
      *point = *point + 1;
      *prev_direction = MS_LEFT;
    }
  }
  
  float max(float a, float b) {
    return a > b ? a : b;
  }

  void ContourSimplification::createMinHeap() {
    if (contours_num_elements_.capacity() < contours_starts_.size()) {
      contours_num_elements_.capacity(contours_starts_.size());
    }
    contours_num_elements_.resize(0);
    if (contours_lengths_.capacity() < contours_starts_.size()) {
      contours_lengths_.capacity(contours_starts_.size());
    }
    contours_lengths_.resize(0); 
    
    // Calculate the per edge length and angle
    for (uint32_t i = 0; i < contours_starts_.size(); i++) {
      float cur_contour_length = 0;
      uint32_t cur_num_contours = 0;
      uint32_t start_contour = contours_starts_[i];
      uint32_t cur_contour = start_contour;
      do {
        Contour* cont = contours_.at(cur_contour);
        cont->calcLengthAndAngle(&contours_);
        cur_contour_length += cont->length;
        cur_contour = cont->next;
        cur_num_contours++;
      } while (cur_contour != start_contour);
      contours_num_elements_.pushBack(cur_num_contours);
      contours_lengths_.pushBack(cur_contour_length);
    }
    
    // Calculate the per edge cost --> the length and angle must be valid before
    // calculating cost
    for (uint32_t i = 0; i < contours_starts_.size(); i++) {
      uint32_t start_contour = contours_starts_[i];
      uint32_t cur_contour = start_contour;
      float cur_length = contours_lengths_[i];
      do {
        Contour* cont = contours_.at(cur_contour);
        cont->calcCost(&contours_, cur_length);
        cur_contour = cont->next;
      } while (cur_contour != start_contour);
    }

    heap_.buildHeap(&contours_);
  }

  void ContourSimplification::cullContours(uint32_t target_contour_count, 
      data_str::Vector<math::Float3>* vertices) {
    if (finished_contours_.capacity() < contours_starts_.size()) {
      finished_contours_.capacity(contours_starts_.size());
    }
    finished_contours_.resize(0);
    for (uint32_t i = 0; i < contours_starts_.size(); i++) {
      finished_contours_.pushBack(false);
    }
    uint32_t num_contours_finished = 0;
    
    while (heap_.size() > 0 && 
           num_contours_finished < finished_contours_.size()) {
      Contour* min_contour = heap_.removeMin();
      // Figure out which contour we are part of:
      uint32_t cur_contour = min_contour->contour_index;
      
      if (min_contour->curr == contours_.at(min_contour->next)->next ||
          contours_num_elements_[cur_contour] <= (target_contour_count+1)) {
        // No more edges left to cull in this contour or we've already hit the 
        // target for this contour
        if (finished_contours_[cur_contour] == false) {
          finished_contours_[cur_contour] = true;
          num_contours_finished++;
        }
      } else {
        // Remove the current contour
        Contour* prev_contour = contours_.at(min_contour->prev);
        Contour* prev_prev_contour = contours_.at(prev_contour->prev);
        Contour* next_contour = contours_.at(min_contour->next);
        
        // Now fix up the linkage
        prev_contour->next = min_contour->next;
        next_contour->prev = min_contour->prev;
        
        // Subtract off the current min_contour length and the prev contour
        // length --> Since these segments will be removed
        contours_lengths_[cur_contour] -= min_contour->length;
        contours_lengths_[cur_contour] -= prev_contour->length;
        
        // Now fix up the edge lengths and angles that changed (only 2 of them)
        prev_contour->calcLengthAndAngle(&contours_);
        next_contour->calcLengthAndAngle(&contours_);
        
        // Add in the new segment length
        contours_lengths_[cur_contour] += prev_contour->length;
        
        // Update the contour segment count
        contours_num_elements_[cur_contour] -= 1;
        
        // Update the costs and fix the heap.
        next_contour->calcCost(&contours_, contours_lengths_[cur_contour]);
        prev_contour->calcCost(&contours_, contours_lengths_[cur_contour]);
        prev_prev_contour->calcCost(&contours_, contours_lengths_[cur_contour]);
        
        // Invalidate the min_contour
        min_contour->invalidateContour();
      }
    }

    // Fix up the contour starts --> We could update as we go along (above), but
    // I think removing all the branching conditionals and just doing this as
    // a post-processing step is easier.
    fixContourStarts();
  }

  void ContourSimplification::printContours() {
    uint32_t j = 0;
    for (uint32_t i = 0; i < contours_starts_.size(); i++) {
      uint32_t cur_contour = contours_starts_[i];
      do {
        Contour* cont = contours_.at(cur_contour);
        printf("Contour %d: <%d --> %d>\n", i, cur_contour, cont->next);
        j++;
        cur_contour = cont->next;
      } while (cur_contour != contours_starts_[i]);
    }
  }
  
  void ContourSimplification::fixContourStarts() {
    // Fix up the contour starts (also ignore contours of length 1):
    uint32_t cur_contour = 0;
    uint32_t num_segments;
    contours_starts_.resize(0);
    contours_num_elements_.resize(0);
    while (cur_contour != contours_.size()) {
      Contour* cont = contours_.at(cur_contour);
      if (cont->next != MAX_UINT32) {
        if (cont->next == cont->curr) {
          cont->invalidateContour();
        } else {
          uint32_t start_contour = cur_contour;
          num_segments = 0;
          contours_starts_.pushBack(start_contour);
          // Now go to the end of the current contour
          do {
            num_segments++;
            Contour* cont = contours_.at(cur_contour);
            cont->contour_index = contours_starts_.size() - 1;
            cur_contour = cont->next;
          } while (contours_.at(cur_contour)->next != start_contour);
          contours_num_elements_.pushBack(num_segments);
        }
      }
      cur_contour++;
    }
  }
  
  void ContourSimplification::saveContourToFile(std::string filename) {
    if (contours_.size() <= 0 || contours_starts_.size() <= 0) {
      printf("saveContourToFile() - No contour to save\n");
      return;
    }
    
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    
    size_t Float3_elem_size = sizeof(contours_[0].v1[0])*3;
    uint32_t num_contours = contours_starts_.size();
    
    // The file will be layed out like this:
    //   1. number of contours (32bit unsigned int)
    //   2. For each contour i:
    //      --> number of segments in contour (32bit unsigned int)
    //   3. For each contour i:
    //      --> For each contour segment j
    //           --> Contour[i].segment[j].x (32bit float)
    //           --> Contour[i].segment[j].y (32bit float)
    //           --> Contour[i].segment[j].z (32bit float)
  
    file.write(reinterpret_cast<const char*>(&num_contours), 
               sizeof(num_contours));
    file.flush();
    for (uint32_t i = 0; i < contours_num_elements_.size(); i++) {
      file.write(reinterpret_cast<const char*>(contours_num_elements_.at(i)), 
                 sizeof(num_contours));
      file.flush();
    }
    for (uint32_t i = 0; i < contours_starts_.size(); i++) {
      uint32_t start_contour = contours_starts_[i];
      uint32_t cur_contour = start_contour;
      do {
        Contour* cont = contours_.at(cur_contour);
        file.write(reinterpret_cast<const char*>(cont->v1.m), Float3_elem_size);
        file.flush();        
        cur_contour = cont->next;
      } while (cur_contour != start_contour);
    }
    file.close();
  }
}  // namespace contour_simplification
