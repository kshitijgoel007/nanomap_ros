^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nanomap_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.0 (2023-11-21)
------------------
* handle zeros from realsense by not adding to kdtree
* Return free space if depth is close to zero
* patch: fixed transform for query point world frame to last pose body frame
* added sequence number
* added ros NanoMapQuery.msg
* updated topic names to stay consistent in launch file and moved cout prints to debug mode
* removed old file
* Nanomap working. Added back O3 optimization and benchmarking node
* corrected frustum visualization FOV
* fixing segfault in AddPose
  wip: Minimal reproducible example of segfault in nanomap_types.h::NanoMapPose constructor
  upgraded to C++ standard 17 -- fixes eigen alignment issue
* change header naming and export conventions
* remove printout
* checked poses size - run of ram/swap
* param from launch file
* split launch files
* refactoring + query local points
* Update README.md
* clang-format style=Google
* Update README.md
* add visualization node
* fix CMakeLists to be able to run tests
* CanSetHistoryLength test
* VerifyCanBufferPointClouds test
* VerifyCanUpdateAll test
* successful no jumps test
* continue with updating
* updating poses with prints in test
* fixed poses.size() + num_deleted bug, still working through VerifyNoJumps test
* great first test -- verifies adding edges
* builds with test
* add contributors
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* png plots
* try to add benchmark plots
* fix readme type
* update README
* add README
* add license
* rid debug prints
* another fix after concatenation optimization
* fix adding edges with new concatentation scheme
* precompute concatentation of transforms
* edge_rotation_only, helps reduce number of matrix sub block operations
* add const& to function
* dont need num_history count
* fix benchmarking num samples
* point_cloud_count++
* trying same point benchmark
* insertion time working
* pose callback in benchmark node
* pose updates in benchmarking node
* set all paramters in benchmarking node
* set camera info in benchmark, hardcoded
* add nanomap insert into benchmarking node
* benchmarking node builds
* builds in mapping docker
* builds with new CMakeLists and package.xml
* good actual duck experiment
* state for 1 cm experiment
* friday 9:10 am: dry run experimetns around corner
* tuned sigma growth
* nanomap-experiments-under-overpass
* rotate rather than translate sigma, and make it non-negative
* 20 m frustum
* params sensor range
* ahh..
* only allow the behind buffer for the first image
* good early out protocol
* correctly prunes on smoothed pose updates
* WaitForTransforms function
* decimate visualization, looks good\
* visuals work
* fixed single frustum view
* draw current body frame 20 times
* visualization Get, now need visualization draw
* visualization tools added, visualizing current fov, needs to visualize history next
* turn off state print.  turn on 3d flight
  Conflicts:
  cfg/motion_selector.yaml
* turn off NOT MAKING PROGRESS. print state. turn off smooth pose listening. disable aabb.
* param N_depth_image_history
* fix rare out of bounds
* actually using pose updates in chain
* poses will be dynamic in this loop
* supports deleting previous poses and updating
* NanoMap handles pose updates, not yet different uncertainties
* add scaffolding for handling pose updates
* turn off prints
* try allowing only first to have deadband
* memory to 150, with fov constraints to match
* also do not enforce up/down fov constraints, since no memory
* turn off body to rdf print
* master settings for fov costs
* R_body_to_rdf print back on
* History of 1.
* try this
* print for sensor transform
* try this print
* Increase history to 150.
* guards against accessing beyond chain size
* N=1 for flight test
* get rid of deadband
* safe aabb
* first aabb implementation
* ignore sensor horizon for first depth image
* fixed transforms
* N=1 looks good
* adjust p_collision for fov scenarios, decrease N to 150
* pose uncertainty growth
* I think looking good? with N=300
* FindTransform function looks fixed
* working on debugging why transforms so large
* debugging transform over time issue
* not quite switched over to FovEvaluator, working through it
* flying on NanoMap N=1;
* need to figure out why nn is returning such different points
* fix so always return closest points
* switched FindTransform.  Can fly around.  Unsure why visualization not working well
* fix rotation of sigma
* fixed return frame to be rdf
* flew fine, pre-switchover
* axis aligned linear covariance
* printing out distance comparisons
* fixed bug in Knn search
* frame_id implemented
* keep resetting rotation matrix
* adds SameAs for NanoMapTime, which fixes interpolation issue
* switch all time comparisons over to GreaterThan function
* NanoMapTime GreaterThan()
* trimming successfully but now need to check all of my timing code
* print town USA
* debug print city
* debug prints everywhere
* printing KnnQuery reply
* remove prints
* adding poses safely
* nanomap wiring for addpose, setcamera, setR
* implements GetRelativeTransformFromTo
* transform util functions in pose_manager
* GetPoseAtTime and Interpolation
* comments on what to do later
* initialization conditions
* initialize fov_evaluator_ptr in constructor of NanoMap
* manage chain size
* fix push/pop implementations
* dont need const return
* SPCC done
* couple more simple functions
* finished KnnQuery
* search and applying transforms
* ApplyTransform and no more 4f
* KnnQuery in SPCC
* SetBodyToRdf
* about to build KnnQuery
* const happy
* fov_evaluator building and passing
* fov_evaluator builds
* fov_evaluator.h scaffolding
* add enum class
* shared ptrs happy
* bring in kd_tree.h and nanoflann.hpp
* typedef PointCloud
* CanInterpolatePoses
* GetMostRecentPoseTime()
* delete memory before time
* readability improvements
* nanomap.cc AddPose
* std::deque buffer for point clouds
* one types file
* builds but about to move around types
* add point cloud buffer and comments
* structured point cloud chain scaffolding
* NanoMap Time
* pose manager scaffolding
* nanomap directory with preliminary scaffolding
* Contributors: FLA-01, Jake Ware, Jonathan Lee, Kshitij Goel, Pete Florence, Wennie Tabib, pdepetris, peteflorence, tiralonghipol
