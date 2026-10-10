<img width="1920" height="1200" alt="image" src="https://github.com/user-attachments/assets/10600407-9155-4b57-8da5-8192b73f70bd" />



Hi everyone! Here is a quick guide to making Open-RMF maps for Ackerman-type AMRs. This includes creating a new map from scratch as well as converting existing maps.
Make sure to enable "reversible" in your navigation config. Also, don't forget to properly define your AMR's Ackerman kinematics using these exact parameters:
• Min turning radius
• Safety margin on radius
• Wheelbase (front–rear axle)
• Track width (left–right)
• Body length
• Body width
• Wall clearance
download or clone it double tap on lane_maker.html it ll open in ur browser u can either make ur own map or use ur exsiting rmf map and get it converted here to ackerman style and export it directly to 0graph 

u might have to adjust few nodes if the radius isnt comprtible so do increse nav goal tolarnace and u can directly export it to o.yaml format i ll be updating fully working editor soon 
