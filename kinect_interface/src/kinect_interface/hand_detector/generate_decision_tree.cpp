//
//  generate_decision_tree.c
//
//  Created by Jonathan Tompson on 7/20/12.
//

#if defined(WIN32) || defined(_WIN32)
  #ifndef _CRT_RAND_S
  #define _CRT_RAND_S
  #endif
  #include <stdlib.h>  // rand_s
#endif
#include <ctime>
#include <cmath>
#include <stdexcept>
#if defined(WIN32) || defined(_WIN32) || defined(__APPLE__)
  #include <string>
#else
  #include <cstring>
#endif
#include <iostream>
#include <mutex>
#include "kinect_interface/hand_detector/generate_decision_tree.h"
#include "kinect_interface/hand_detector/common_tree_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_func.h"
#include "kinect_interface/hand_detector/evaluate_decision_forest.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "kinect_interface/depth_image_data.h"
#include "jtil/data_str/circular_buffer.h"

using std::string;
using std::runtime_error;
using jtil::data_str::CircularBuffer;
using std::cout;
using std::endl;

namespace kinect_interface {
namespace hand_detector {

  struct QueueNode {
    int32_t tree_node;
    int16_t height;
    int32_t occupancy_start;
    int32_t occupancy_length;
    float entropy;
    QueueNode& operator=(const QueueNode &rhs) {
      // Only do assignment if RHS is a different object from this.
      if (this != &rhs) {
        this->tree_node = rhs.tree_node;
        this->height = rhs.height;
        this->occupancy_start = rhs.occupancy_start;
        this->occupancy_length = rhs.occupancy_length;
        this->entropy = rhs.entropy;
      }
      return *this;
    }
  };

  uint32_t rand_threadsafe(const unsigned int& thread_specific_seed) {
#if defined(WIN32) || defined(_WIN32)
    unsigned int rand_num;
    if (rand_s(&rand_num) != 0) { 
      throw std::runtime_error("rand_s returned an error!"); 
    }
    return static_cast<uint32_t>(rand_num);
#else
    return rand_r(&thread_specific_seed);
#endif
  }

  int32_t GenerateDecisionTree::populateOccupancyList(
    const DepthImageData& data, const int32_t max_pix_per_image, 
    const unsigned int& seed) {
    // First count the number of starting pixels, these are pixels that are NOT
    // '0' (which is NEVER a hand) and which are too far away.
    int32_t im_size = data.im_width * data.im_height;
    int32_t label_count_cur_im[NUM_LABELS];
    int32_t num_points = 0;
    for (int32_t i = 0; i < data.num_images; i++) {
      memset(label_count_cur_im, 0, NUM_LABELS * sizeof(label_count_cur_im[0]));
      // Get the label distribution for this image
      for (int32_t pix = 0; pix < im_size; pix++) {
        int32_t cur_index = (i * im_size) + pix;
        if (data.image_data[cur_index] != 0 && data.image_data[cur_index] < GDT_MAX_DIST) {
          uint8_t cur_label = data.label_data[cur_index];
  #if defined(DEBUG) || defined(_DEBUG)
          if (cur_label >= NUM_LABELS) {
            throw std::runtime_error("ERROR: cur_label >= NUM_LABELS!");
          }
  #endif
          label_count_cur_im[cur_label]++;
        }
      }
      // Add the label distribution to the total image
      for (uint32_t lab = 0; lab < NUM_LABELS; lab++) {
        int32_t count = label_count_cur_im[lab] > max_pix_per_image ?
          max_pix_per_image : label_count_cur_im[lab];
        num_points += count;
      }
    }

    cur_occ_list = new int32_t[num_points];
    next_occ_list = new int32_t[num_points];
    int32_t* temp_space = new int32_t[im_size * NUM_LABELS];
  
    uint32_t index = 0;
    // Now go through again adding all those points to the starting occ list
    for (int32_t i = 0; i < data.num_images; i++) {
      memset(label_count_cur_im, 0, NUM_LABELS * sizeof(label_count_cur_im[0]));
      // Get the label distribution for this image again, but this time build a 
      // temporary occ list for it
      for (int32_t pix = 0; pix <im_size; pix++) {
        int32_t cur_index = (i * im_size) + pix;
        if (data.image_data[cur_index] != 0 && data.image_data[cur_index] < GDT_MAX_DIST) {
          uint8_t cur_label = data.label_data[cur_index];
          temp_space[(cur_label * im_size) + label_count_cur_im[cur_label]] = cur_index;
          label_count_cur_im[cur_label]++;
        }
      }
      // Perform a Knuth O(n) shuffle, but stop after the first max_pix_per_image
      for (uint32_t lab = 0; lab < NUM_LABELS; lab++) {
        int32_t count = label_count_cur_im[lab] > max_pix_per_image ?
          max_pix_per_image : label_count_cur_im[lab];
        for (int32_t pix = 0; pix < (count - 1); pix++) {
          // Random integer from [pix, label_count_cur_im[lab]]
          uint32_t rand_index = pix + (rand_threadsafe(seed) % (label_count_cur_im[lab] - pix));
  #if defined(DEBUG) || defined(_DEBUG)
          if (rand_index >= static_cast<uint32_t>(label_count_cur_im[lab]) || 
            rand_index < static_cast<uint32_t>(pix)) {
            throw std::runtime_error("ERROR: rand_index >= count!"); 
          }
  #endif
          int32_t swap_val = temp_space[(lab * im_size) + pix];
          temp_space[(lab * im_size) + pix] = temp_space[(lab * im_size) + rand_index];
          temp_space[(lab * im_size) + rand_index] = swap_val;
        }
        // Add them to the final set
        for (int32_t pix = 0; pix < count; pix++) {
          cur_occ_list[index] = temp_space[(lab * im_size) + pix];
          index++;
          if (index > static_cast<uint32_t>(num_points)) {
            throw std::runtime_error("ERROR: index > num_points!");
          }
        }
      }
    }

    delete[] temp_space;

    return num_points;
  }

  //**********************************************************
  //****************** MAIN ENTRY POINT **********************
  //**********************************************************
  void GenerateDecisionTree::generateDecisionTree(DecisionTree& dt,
    const DepthImageData& train_data, const WLSet& wl_set, 
    const TrainingSettings& settings) {

    // Check input 
    uint32_t tree_size = calcTreeSize(settings.tree_height);
    uint64_t data_length = settings.num_im_to_consider * 
      train_data.im_width * train_data.im_height;
    if (data_length > 0x7fffffff) {  // MAX 32bit int
      throw std::wruntime_error("GenerateDecisionTree::generateDecisionTree() "
        "- ERROR: Data length is > int32_t address space!");
    }

    dt.tree_height = settings.tree_height;
    dt.num_nodes = 0;
    dt.tree = new DecisionTreeNode[tree_size];  // Max size
    for (uint32_t i = 0; i < tree_size; i++) {
      dt.tree[i].left_child = -1;
      dt.tree[i].right_child = -1;
    }

  #ifdef VERBOSE_GENERATION
    cout << "generateDecisionTree INPUTS:" << endl;
    cout << "   num_images = " << settings.num_images_to_consider << endl;
    cout << "   wl_coeffs_sizes = [" << wl_set.wl_coeffs_sizes[0] <<
            ", " << wl_set.wl_coeffs_sizes[1] <<
            ", " << wl_set.wl_coeffs_sizes[2] << "]" << endl;
    cout << "   num_samples_per_node = " << wl_set.num_samples_per_node << endl;
    cout << "   tree_height = " << settings.tree_height << endl;
    cout << "   tree_size = " << tree_size << endl;
    cout << "   width = " << train_data.im_width << endl;
    cout << "   height = " << train_data.im_height << endl;
    cout << "   tree# = " << settings.dt_index << endl;
  #endif

    // Create the queue structure --> Use for BFS of the Decision Tree
    uint32_t queue_size = (tree_size + 1) / 2;
    CircularBuffer<QueueNode>* queue = new CircularBuffer<QueueNode>(queue_size);
    QueueNode root;
    root.tree_node = 0;
    root.height = 1;
    root.occupancy_start = 0;

    cout << endl << "DT#" << settings.dt_index;
    cout << ": initializing occupancy list..." << endl;

    // Create the occupancy structures.  Since we're desending the tree BFS we
    // need to store the occupancy list only for two levels of the tree.  We'll
    // then ping-pong back and forth between two occupancy buffers.
    root.occupancy_length = populateOccupancyList(train_data, 
      settings.max_pix_per_im_per_label, settings.seed);

    uint32_t num_wl_permutations = 1;
    uint32_t num_samples_per_node = wl_set.num_samples_per_node;
    for (uint32_t i = 0; i < 3; i++) {
      num_wl_permutations *= wl_set.wl_coeffs_sizes[i];
    }
    if (num_samples_per_node > num_wl_permutations) {
      num_samples_per_node = num_wl_permutations;
    }
    if (settings.tree_height > num_wl_permutations) {
      throw runtime_error(string("generateDecisionTree:unexpectedData") + 
        string("The weak learner set is smaller than the height of the tree!"));
    }

    // Calculate the entropy of the root and at the same time pack the 
    // occupancy image.
  #ifdef VERBOSE_GENERATION
    cout << "calculating root entropy..." << endl;
  #endif
    uint32_t hist_root[NUM_LABELS];
    memset(hist_root, 0, NUM_LABELS * sizeof(hist_root[0]));
    for (int32_t i = 0; i < root.occupancy_length; i++) {
      hist_root[train_data.label_data[cur_occ_list[i]]]++;
    }
    float sum_hist = static_cast<float>(root.occupancy_length);
    float p_root[NUM_LABELS];
    for (uint32_t i = 0; i < NUM_LABELS; i++) {
      p_root[i] = static_cast<float>(hist_root[i]) / sum_hist;
      dt.tree[0].prob[i] = p_root[i];
    }
    root.entropy = calcEntropy(p_root);
    dt.num_nodes++;

    // Now we have fully specified the root node, push it onto the queue
    queue->write(root);

  #ifdef VERBOSE_GENERATION
    cout << "  hist_root = <";
    for (uint32_t i = 0; i < NUM_LABELS; i++) {
      cout << dt.tree[0].prob[i];
      if (i != (NUM_LABELS - 1)) { cout << ", "; }
    }
    cout << ">" << endl << "  root.occupancy_length = " << root.occupancy_length;
    cout << endl << "  root.entropy = " << root.entropy << endl << endl;
  #endif

    //***************************************************
    //****************** MAIN LOOP **********************
    //***************************************************
    uint32_t num_nodes_finished = 0;
    uint32_t hist_left[NUM_LABELS];
    uint32_t hist_right[NUM_LABELS];
    uint32_t hist_left_best[NUM_LABELS];
    uint32_t hist_right_best[NUM_LABELS];
    float prob_left[NUM_LABELS];
    float prob_right[NUM_LABELS];
    uint32_t cur_height = 1;
    QueueNode queue_left_node, queue_right_node, queue_cur_node;
    float max_gain;
    uint32_t num_attempts;
    DecisionTreeNode* tree_cur_node;
    while (!queue->empty()) {
      // Get the next node off the queue
      queue->read(queue_cur_node);
      bool changed_height = (int16_t)(cur_height) != queue_cur_node.height;
      if (changed_height) {
        // The BFS moved down a level, ping-pong the occupancy list buffers
        int32_t* tmp = cur_occ_list;
        cur_occ_list = next_occ_list;
        next_occ_list = tmp;
        cur_height = queue_cur_node.height;
      }
      int32_t* occ_sub_list = &cur_occ_list[queue_cur_node.occupancy_start];

      // Add 2 children to the data struct
      tree_cur_node = &dt.tree[queue_cur_node.tree_node];
      tree_cur_node->left_child = dt.num_nodes;
      dt.num_nodes++;
      tree_cur_node->right_child = dt.num_nodes;
      dt.num_nodes++;

  #ifndef VERBOSE_GENERATION
      if (cur_height == 1 || changed_height)  {
  #endif
        cout << endl << "DT#" << settings.dt_index << ": Processing node ";
        cout << num_nodes_finished+1 << " of " << tree_size << " (tree index = ";
        cout << queue_cur_node.tree_node << ", height " << queue_cur_node.height << " of ";
        cout << settings.tree_height << ", occp. length = ";
        cout << queue_cur_node.occupancy_length << ")" << endl;
  #ifdef VERBOSE_GENERATION
        cout << "   trying weak learner (of " << num_samples_per_node;
        cout << " attempts): 1 ";
  #endif
  #ifndef VERBOSE_GENERATION
      }
  #endif

      // Now, from the list of still avaliable WL permutations choose a maximal
      // WL from a random subset
      max_gain = 0.0f;
      num_attempts = 0;
      while ((num_attempts < num_samples_per_node) && 
             (max_gain < settings.min_info_gain)) {
        num_attempts++;
  #ifdef VERBOSE_GENERATION
        if ((num_attempts % 100) == 0) { cout << num_attempts << " "; }
  #endif
        // Pick a random coefficient from each coefficient set
        // Note: we must use rand_r (or rand_s on Win32) with our own PRNG state
        // per thread
      
        int32_t cur_wlu_offset = wl_set.wl_coeffs0[rand_threadsafe(settings.seed)
          % wl_set.wl_coeffs_sizes[0]];
        int32_t cur_wlv_offset = wl_set.wl_coeffs1[rand_threadsafe(settings.seed)
          % wl_set.wl_coeffs_sizes[1]];
        int16_t cur_threshold = wl_set.wl_coeffs2[rand_threadsafe(settings.seed)
          % wl_set.wl_coeffs_sizes[2]];
        uint8_t cur_wl_func = wl_set.wl_funcs[rand_threadsafe(settings.seed)
          % wl_set.wl_coeffs_sizes[3]];

        // For all of the data points still alive, bin all of the data points
        // using the weak lerner
        memset(hist_left, 0, sizeof(hist_left[0]) * NUM_LABELS);
        memset(hist_right, 0, sizeof(hist_right[0]) * NUM_LABELS);
        for (int32_t i = 0; i < queue_cur_node.occupancy_length; i++) {
          // We need to calculate back the u, v so that we can add the appropriate
          // offset for this weak learner
          int32_t index = occ_sub_list[i];
          bool result;
          WL_FUNC(index, cur_wlu_offset, cur_wlv_offset, cur_threshold, 
            cur_wl_func, train_data.im_width, train_data.im_height, 
            train_data.image_data, result);

          if (result) {
            // Go left
            hist_left[train_data.label_data[index]]++;
          } else {
            // Go right
            hist_right[train_data.label_data[index]]++;
          }
        }  // for (int32_t i = 0; i < cur_node.occupancy_length; i++)

        // Only continue on if we made some sort of split.
        float num_nodes_left = 0;
        float num_nodes_right = 0;
        for (uint32_t i = 0; i < NUM_LABELS; i++) {
          num_nodes_left += static_cast<float>(hist_left[i]);
          num_nodes_right += static_cast<float>(hist_right[i]);
        }
        if (num_nodes_left > 0 && num_nodes_right > 0) {
          // Normalize the histograms (to calculate probability)
          for (uint32_t i = 0; i < NUM_LABELS; i++) {
            prob_left[i] = static_cast<float>(hist_left[i]) / num_nodes_left;
            prob_right[i] = static_cast<float>(hist_right[i]) / num_nodes_right;
          }

          // from the left and right histograms calculate the entropy
          float entropy_left = calcEntropy(prob_left);
          float entropy_right = calcEntropy(prob_right);

          // Calculate the normalized information gain (using shannon entropy)
          // From Murphy's matlab code (thanks Papa Murphy!):
          // gain = h0 - (h1 * s1/s0 + h2 * s2/s0);
          float cur_gain = queue_cur_node.entropy - 
            (entropy_left * num_nodes_left/(float)(queue_cur_node.occupancy_length) +
            entropy_right * num_nodes_right/(float)(queue_cur_node.occupancy_length));

          if (cur_gain > max_gain) {
            max_gain = cur_gain;
            tree_cur_node->coeff0 = cur_wlu_offset;
            tree_cur_node->coeff1 = cur_wlv_offset;
            tree_cur_node->coeff2 = cur_threshold;
            tree_cur_node->wl_func = cur_wl_func;
            memcpy(hist_left_best, hist_left, NUM_LABELS * sizeof(hist_left_best[0]));
            memcpy(hist_right_best, hist_right, NUM_LABELS * sizeof(hist_right_best[0]));
            queue_left_node.entropy = entropy_left;
            queue_right_node.entropy = entropy_right;
          }
        } // if ((hist_left[0] + hist_left[1]) != 0 && (hist_right[0] + hist_right[1]) != 0) {
      }  // while (num_attemps < WL_samples_per_node)

      if (max_gain > 0) {
        // Now we have the best WL, calculate the occupancy again --> This Repeats
        // some calculations above, but we could potentially be looking at 1000s of
        // WLs, so it should save some time in the long run (since we don't have
        // to save the entire occupancy list every time we find a new best WL).
        queue_left_node.occupancy_length = 0;
        queue_right_node.occupancy_length = 0;
        for (uint32_t i = 0; i < NUM_LABELS; i++) {
          queue_left_node.occupancy_length += hist_left_best[i];
          queue_right_node.occupancy_length += hist_right_best[i];
        }
        queue_left_node.occupancy_start = queue_cur_node.occupancy_start;   
        queue_right_node.occupancy_start = queue_left_node.occupancy_start + queue_left_node.occupancy_length;

        int32_t* occ_sub_list_left = &next_occ_list[queue_left_node.occupancy_start];
        int32_t* occ_sub_list_right = &next_occ_list[queue_right_node.occupancy_start];
        int32_t cur_wlu_offset = tree_cur_node->coeff0;
        int32_t cur_wlv_offset = tree_cur_node->coeff1;
        int16_t cur_threshold = tree_cur_node->coeff2;
        uint8_t cur_wl_func = tree_cur_node->wl_func;
        for (int32_t i = 0; i < queue_cur_node.occupancy_length; i++) {
          // We need to calculate back the u, v so that we can add the appropriate
          // offset for this weak learner
          int32_t index = occ_sub_list[i];
          bool result;
          WL_FUNC(index, cur_wlu_offset, cur_wlv_offset, cur_threshold, 
            cur_wl_func, train_data.im_width, train_data.im_height, 
            train_data.image_data, result);

          if (result) {
            // Go left
            *occ_sub_list_left = index;
            occ_sub_list_left++;
          } else {
            // Go right
            *occ_sub_list_right = index;
            occ_sub_list_right++;
          }
        }  // for (int32_t i = 0; i < stack_occupancy_lengths[cur_stack_ptr]; i++)

  #ifdef VERBOSE_GENERATION
        cout << endl << "    Best weak lerner: <" << tree_cur_node->coeff0;
        cout << ", " << tree_cur_node->coeff1;
        cout << ", " << tree_cur_node->coeff2;
        cout << ", " << static_cast<int>(tree_cur_node->wl_func) << "> with info gain ";
        cout << max_gain << endl;
        cout << "    cur_node.entropy = " << queue_cur_node.entropy << endl;
        cout << "    cur_node.occupancy_length = " << queue_cur_node.occupancy_length << endl;
        cout << "    hist_left = <";
        for (uint32_t i = 0; i < NUM_LABELS; i++) {
          cout << hist_left_best[i];
          if (i != (NUM_LABELS - 1)) { cout << ", "; }
        }
        cout << ">" << endl << "    hist_right = <";
        for (uint32_t i = 0; i < NUM_LABELS; i++) {
          cout << hist_right_best[i];
          if (i != (NUM_LABELS - 1)) { cout << ", "; }
        }
        cout << ">" << endl;
  #endif
        queue_left_node.tree_node = tree_cur_node->left_child;
        queue_right_node.tree_node = tree_cur_node->right_child;
        queue_left_node.height = queue_cur_node.height + 1;
        queue_right_node.height = queue_cur_node.height + 1;

        // Normalize the histograms (to calculate probability)
        for (uint32_t i = 0; i < NUM_LABELS; i++) {
          dt.tree[queue_left_node.tree_node].prob[i] = 
            static_cast<float>(hist_left_best[i]) / static_cast<float>(queue_left_node.occupancy_length);
          dt.tree[queue_right_node.tree_node].prob[i] = 
            static_cast<float>(hist_right_best[i]) / static_cast<float>(queue_right_node.occupancy_length);;
        }

        // Are we a leaf? If we're not, then put our children and the necessary data
        // on the stack
        if (cur_height < (settings.tree_height - 1)) {
          // If any of the histograms of our children are fully split (that is,
          // if one of the labels has PR(label) == 1, then we shouldn't put it
          // on the stack for processing.
          bool add_left = queue_left_node.occupancy_length != 0;
          for (uint32_t i = 0; i < NUM_LABELS && add_left; i++) {
            if ((int32_t)(hist_left_best[i]) == queue_left_node.occupancy_length) {
              add_left = false;
            }
          }
          bool add_right = queue_right_node.occupancy_length != 0;
          for (uint32_t i = 0; i < NUM_LABELS && add_right; i++) {
            if ((int32_t)(hist_right_best[i]) == queue_right_node.occupancy_length) {
              add_right = false;
            }
          }
          // Add left child to the stack for processing
          if (add_left) {
            queue->write(queue_left_node);
          } else {
            // Number of sub-children we will be skipping over
            num_nodes_finished += (1<<(settings.tree_height - cur_height)) - 1;
          }

          // Add right child to the stack for processing
          if (add_right) {
            queue->write(queue_right_node);
          } else {
            // Number of sub-children we will be skipping over
            num_nodes_finished += (1<<(settings.tree_height - cur_height)) - 1;
          }
        } else {
          num_nodes_finished += 2;  // To count for the leaf nodes we finished
        }
      
      } else {  // if (max_gain > 0)
  #ifdef VERBOSE_GENERATION
        cout << "    Couldn't find a weak learner with gain > 0!" << endl;
        cout << "    cur_node hist = <";
        for (uint32_t i = 0; i < NUM_LABELS; i++) {
          cout << tree_cur_node->prob[i];
          if (i != (NUM_LABELS - 1)) { cout << ", "; }
        }
        cout << ">" << endl;
  #endif
        num_nodes_finished += 2*((1<<(settings.tree_height - cur_height)) - 1);
      }
      num_nodes_finished++;
    }  // while (cur_stack_ptr >= 0)

    // Clean up on finish
    delete queue;
    delete[] cur_occ_list;
    delete[] next_occ_list;

    cout << endl << "DT#" << settings.dt_index << ": Tree finished" << endl;
  }

}  // namespace hand_detector
}  // namespace kinect_interface