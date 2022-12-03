import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('ori_matsim_map',help='original MATSim map file',type=str)
    args = parser.parse_args()