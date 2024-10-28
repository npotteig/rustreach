import pandas as pd

if __name__ == '__main__':
    stat_path = 'eval_output_data/bicycle/line_eval_output_rtreach_dyn_5.csv'
    df = pd.read_csv(stat_path)
    
    ttg_df = df[(df['TTG'] != -1) | (df['TTG'] <= 20.0)]
    print("Mean TTG: ", ttg_df['TTG'].mean())
    
    total_steps = df['TTG'].sum() / 0.1
    print("Total Steps: ", total_steps)
    
    deadline_df = df[df['Deadline Violations'] == 1]
    print("Deadline Violations: ", len(deadline_df))
    
    print("Deadline Violations w/ No Subgoal Found: ", len(deadline_df[deadline_df['No Subgoal'] == 1]))
    
    print("Collision & No Subgoal: ", len(df[(df['Collision'] == 1) & (df['No Subgoal'] == 1)]))