import pandas as pd

TOTAL_TIME = 100.0

if __name__ == '__main__':
    stat_path = 'eval_output_data/bicycle/path_exp/path_eval_output_rtreach_dyn_10.csv'
    df = pd.read_csv(stat_path)
    
    ttg_df = df[(df['TTG'] != -1) | (df['TTG'] < TOTAL_TIME)]
    print("Mean TTG: ", ttg_df['TTG'].mean(), "s")
    
    timouts_df = df[df['TTG'] > TOTAL_TIME]
    print("Timeouts: ", len(timouts_df))
    
    collision_df = df[df['Collision'] == 1]
    print("Collisions: ", len(collision_df))
    
    no_subgoal_df = df[df['No Subgoal'] == 1]
    print("No Subgoal: ", len(no_subgoal_df))
    
    avg_sg_compute_time = df['Avg Subgoal Compute Time'].mean()
    print("Avg Subgoal Compute Time: ", avg_sg_compute_time, "us")
    
    max_sg_compute_time = df['Max Subgoal Compute Time'].max()
    print("Max Subgoal Compute Time: ", max_sg_compute_time, "us")
    
    deadline_df = df[df['Deadline Violations'] == 1]
    print("Deadline Violations: ", len(deadline_df))