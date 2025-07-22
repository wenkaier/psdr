import argparse
import subprocess
import os
import os.path as osp
from pypsdr import psdr

def run_psdr(input_ply_path, export_folder_path, epsilon_ratio=0.01, min_inliers=50, knn=10, normal_th=0.8, refine_seconds=180):
    # initialise a planar shape detector                                    
    ps = psdr(verbosity=1)              

    # load input point cloud                                         
    ps.load_points(input_ply_path)
    bb_diagonal = ps.get_bounding_box_diagonal()

    # detect planar shapes with fitting tolerance epsilon = 1% of the pointcloud's bounding box diagonal
    ps.detect(epsilon=epsilon_ratio*bb_diagonal,min_inliers=min_inliers,knn=knn,normal_th=normal_th)

    # refine planar shape configuration until convergence (i.e. no limit on number of iterations)
    ps.refine(max_iterations=-1)
    # if the point cloud is very large (e.g. > 2M points) you can also set a time limit
    ps.refine(max_seconds=refine_seconds)
    # export_folder_path
    # export planar shapes
    ps.save(osp.join(export_folder_path,"convexes.ply"),"convex")                  
    ps.save(osp.join(export_folder_path,"rectangles.ply"),"rectangles")            
    ps.save(osp.join(export_folder_path,"alpha_shapes.ply"),"alpha")               
    ps.save(osp.join(export_folder_path,"point_groups.ply"),"pointcloud")               
    ps.save(osp.join(export_folder_path,"point_groups.vg"))                              
    ps.save(osp.join(export_folder_path,"point_groups.npz"))                              

def main():
    parser = argparse.ArgumentParser(description="Call psdr-cli from command line")

    parser.add_argument("--input_ply_path", type=str, required=True, help="Input point cloud (PLY)")
    parser.add_argument("--export_folder_path", type=str, required=True, help="Directory to save outputs")
    parser.add_argument("--epsilon_ratio", type=float, default=0.01, help="Epsilon as ratio of bounding box diagonal")
    parser.add_argument("--min_inliers", type=int, default=50, help="Minimum inliers")
    parser.add_argument("--knn", type=int, default=10, help="KNN for neighbor graph")
    parser.add_argument("--normal_th", type=float, default=0.8, help="Normal similarity threshold")
    parser.add_argument("--refine_seconds", type=int, default=180, help="Max seconds for refinement")

    args = parser.parse_args()
    os.makedirs(args.export_folder_path, exist_ok=True)
    
    run_psdr(args.input_ply_path, args.export_folder_path, 
             epsilon_ratio=args.epsilon_ratio, min_inliers=args.min_inliers,
             knn=args.knn, normal_th=args.normal_th, refine_seconds=args.refine_seconds)

if __name__ == "__main__":
    main()
