import pickle

from darp import DARP
import numpy as np
from kruskal import Kruskal
from CalculateTrajectories import CalculateTrajectories
from Visualization import visualize_paths
import sys
import argparse
from turns import turns
from PIL import Image
import time

def get_area_map(path, row, col):
    load_img_rz = np.array(Image.open(path).resize((row,col)))
    grid_map = np.zeros([col, row], dtype = int)

    for i in range(len(load_img_rz)):
        for j in range(len(load_img_rz[0])):
            if list(load_img_rz[i][j]) == [255, 255, 255]:
                grid_map[i][j] = 0
            else:
                grid_map[i][j] = 1

    return grid_map



# def get_area_indices(area, value, inv=False, obstacle=-1):
#     """
#     Returns area tiles indices that have value
#     If inv(erted), returns indices that don't have value
#     :param area: array with value and obstacle tiles
#     :param value: searched tiles with value
#     :param inv: if True: search will be inverted and index of non-value tiles will get returned
#     :param obstacle: defines obstacle tiles
#     :return:
#     """
#     try:
#         value = int(value)
#         if inv:
#             return np.concatenate([np.where((area != value))]).T
#         return np.concatenate([np.where((area == value))]).T
#     except:
#         mask = area == value[0]
#         if inv:
#             mask = area != value[0]
#         for v in value[1:]:
#             if inv:
#                 mask &= area != v
#             else:
#                 mask |= area == v
#         mask &= area != obstacle
#         return np.concatenate([np.where(mask)]).T

class MultiRobotPathPlanner(DARP):
    def __init__(self,
                grid_row=10,
                grid_column=10,
                agent_initial_positions=[0, 3, 9],
                obstacle_positions=[5, 6, 7],
                equal_portions=False,
                portions=[0.2, 0.3, 0.5],
                visualization=False,
                MaxIter=80000,
                CCvariation=0.01,
                randomLevel=0.0001,
                dcells=2,
                importance=False):

        start_time = time.time()
        # Initialize DARP
        self.darp_instance = DARP(grid_row,
                                grid_column,
                                agent_initial_positions,
                                obstacle_positions,
                                equal_portions,
                                portions,
                                visualization,
                                MaxIter=MaxIter,
                                CCvariation=CCvariation,
                                randomLevel=randomLevel,
                                dcells=dcells,
                                importance=importance)

        # Divide areas based on robots initial positions
        self.DARP_success , self.iterations = self.darp_instance.divideRegions()

        # Check if solution was found
        if not self.DARP_success:
            print("DARP did not manage to find a solution for the given configuration!")
            print("iterations...", self.iterations)
        else:
            print("self.iterations", self.iterations)
            # Iterate for 4 different ways to join edges in MST
            self.mode_to_drone_turns = []
            AllRealPaths_dict = {}
            subCellsAssignment_dict = {}
            for mode in range(4):
                MSTs = self.calculateMSTs(self.darp_instance.BinaryRobotRegions, self.darp_instance.droneNo, self.darp_instance.rows, self.darp_instance.cols, mode)
                AllRealPaths = []
                for r in range(self.darp_instance.droneNo):
                    ct = CalculateTrajectories(self.darp_instance.rows, self.darp_instance.cols, MSTs[r])
                    ct.initializeGraph(self.CalcRealBinaryReg(self.darp_instance.BinaryRobotRegions[r], self.darp_instance.rows, self.darp_instance.cols), True)
                    ct.RemoveTheAppropriateEdges()
                    ct.CalculatePathsSequence(4 * self.darp_instance.initial_positions[r][0] * self.darp_instance.cols + 2 * self.darp_instance.initial_positions[r][1])
                    AllRealPaths.append(ct.PathSequence)

                self.TypesOfLines = np.zeros((self.darp_instance.rows*2, self.darp_instance.cols*2, 2))
                for r in range(self.darp_instance.droneNo):
                    flag = False
                    for connection in AllRealPaths[r]:
                        if flag:
                            if self.TypesOfLines[connection[0]][connection[1]][0] == 0:
                                indxadd1 = 0
                            else:
                                indxadd1 = 1

                            if self.TypesOfLines[connection[2]][connection[3]][0] == 0 and flag:
                                indxadd2 = 0
                            else:
                                indxadd2 = 1
                        else:
                            if not (self.TypesOfLines[connection[0]][connection[1]][0] == 0):
                                indxadd1 = 0
                            else:
                                indxadd1 = 1
                            if not (self.TypesOfLines[connection[2]][connection[3]][0] == 0 and flag):
                                indxadd2 = 0
                            else:
                                indxadd2 = 1

                        flag = True
                        if connection[0] == connection[2]:
                            if connection[1] > connection[3]:
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 2
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 3
                            else:
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 3
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 2

                        else:
                            if (connection[0] > connection[2]):
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 1
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 4
                            else:
                                self.TypesOfLines[connection[0]][connection[1]][indxadd1] = 4
                                self.TypesOfLines[connection[2]][connection[3]][indxadd2] = 1

                subCellsAssignment = np.zeros((2*self.darp_instance.rows, 2*self.darp_instance.cols))
                for i in range(self.darp_instance.rows):
                    for j in range(self.darp_instance.cols):
                        subCellsAssignment[2 * i][2 * j] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i][2 * j + 1] = self.darp_instance.A[i][j]
                        subCellsAssignment[2 * i + 1][2 * j + 1] = self.darp_instance.A[i][j]

                drone_turns = turns(AllRealPaths)
                drone_turns.count_turns()
                drone_turns.find_avg_and_std()
                self.mode_to_drone_turns.append(drone_turns)

                AllRealPaths_dict[mode] = AllRealPaths
                subCellsAssignment_dict[mode] = subCellsAssignment


            # Find mode with the smaller number of turns
            averge_turns = [x.avg for x in self.mode_to_drone_turns]
            self.min_mode = averge_turns.index(min(averge_turns))
            
            # Retrieve number of cells per robot for the configuration with the smaller number of turns
            min_mode_num_paths = [len(x) for x in AllRealPaths_dict[self.min_mode]]
            min_mode_returnPaths = AllRealPaths_dict[self.min_mode]

            # Uncomment if you want to visualize all available modes
            # if self.darp_instance.visualization:
            #     for mode in range(4):
            #         image = visualize_paths(AllRealPaths_dict[mode], subCellsAssignment_dict[mode],
            #                                 self.darp_instance.droneNo, self.darp_instance.color)
            #         image.visualize_paths(mode)
            #     print("Best Mode:", self.min_mode)

            #Combine all modes to get one mode with the least available turns for each drone
            combined_modes_paths = []
            combined_modes_turns = []
            
            for r in range(self.darp_instance.droneNo):
                min_turns = sys.maxsize
                temp_path = []
                for mode in range(4):
                    if self.mode_to_drone_turns[mode].turns[r] < min_turns:
                        temp_path = self.mode_to_drone_turns[mode].paths[r]
                        min_turns = self.mode_to_drone_turns[mode].turns[r]
                combined_modes_paths.append(temp_path)
                combined_modes_turns.append(min_turns)

            self.best_case = turns(combined_modes_paths)
            self.best_case.turns = combined_modes_turns
            self.best_case.find_avg_and_std()
            
            # Retrieve number of cells per robot for the best case configuration
            best_case_num_paths = [len(x) for x in self.best_case.paths]
            best_case_returnPaths = self.best_case.paths
            
            #visualize best case
            if self.darp_instance.visualization:
                image = visualize_paths(self.best_case.paths, subCellsAssignment_dict[self.min_mode],
                                        self.darp_instance.droneNo, self.darp_instance.color)
                image.visualize_paths("Combined Modes")

            self.execution_time = time.time() - start_time
            
            print(f'\nResults:')
            print(f'Number of cells per robot: {best_case_num_paths}')
            print(f'Minimum number of cells in robots paths: {min(best_case_num_paths)}')
            print(f'Maximum number of cells in robots paths: {max(best_case_num_paths)}')
            print(f'Average number of cells in robots paths: {np.mean(np.array(best_case_num_paths))}')
            print(f'\nTurns Analysis: {self.best_case}')
            print(f'\nTime it take: {self.execution_time}')
            
    def CalcRealBinaryReg(self, BinaryRobotRegion, rows, cols):
        temp = np.zeros((2*rows, 2*cols))
        RealBinaryRobotRegion = np.zeros((2 * rows, 2 * cols), dtype=bool)
        for i in range(2*rows):
            for j in range(2*cols):
                temp[i, j] = BinaryRobotRegion[(int(i / 2))][(int(j / 2))]
                if temp[i, j] == 0:
                    RealBinaryRobotRegion[i, j] = False
                else:
                    RealBinaryRobotRegion[i, j] = True

        return RealBinaryRobotRegion

    def calculateMSTs(self, BinaryRobotRegions, droneNo, rows, cols, mode):
        MSTs = []
        for r in range(droneNo):
            k = Kruskal(rows, cols)
            k.initializeGraph(BinaryRobotRegions[r, :, :], True, mode)
            k.performKruskal()
            MSTs.append(k.mst)
        return MSTs

if __name__ == '__main__':
    MultiRobotPathPlanner(
                            grid_row=5,
                            grid_column=5,
                            agent_initial_positions=[0, 2, 4],
                            obstacle_positions=[7, 8],
                            # equal_portions=False,
                            # portions=[0.2, 0.3, 0.5],
                            visualization=False,
                            # MaxIter=80000,
                            # CCvariation=0.01,
                            # randomLevel=0.0001,
                            # dcells=2,
                            # importance=False
    )
    
    turned_data = get_area_map("/media/hmk/Files/Projects/DARP-Conda/images/2_640_480.png", 32, 24)
    print(turned_data)
    