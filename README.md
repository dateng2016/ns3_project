## READY TO RUN
All the necessary files are already included in the final project folder including all the input and output files. Copy everything inside the `scratch` folder within the final project folder into your ns3 scratch folder. Go to your ns3 root directory and run the following command:
`./ns3 run scratch/final_project.cc -- --location=filteredGTLow --protocol=AODV`
The allowed inputs for `location` are `filteredGTLow, filteredGTHigh, filteredFreTexas, filteredTimesSquareHigh`. The allowed inputs for `protocol` are `AODV, DSDV, OLSR`.  

After successfully running the ns3 simulation, open up the Jupyter Notebook parse_results.ipynb and run all the cells. This will generate all the results needed for this project.
