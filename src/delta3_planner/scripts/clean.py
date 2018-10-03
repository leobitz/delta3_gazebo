import numpy as np

def get_error(data):
    errors = 0
    for d in range(len(data) -1 ):
        row = data[d]
        i = int(row[0]) - 1
        expected = data[d][1:4]
        actual = data[d + 1][4:7]
        expected[i] = 0
        actual[i] = 0
        error =  np.sqrt(np.mean((expected - actual) ** 2))
        errors += error
    print(errors/(len(data) -1))

def get_data():
    lines = open('mydata.txt').readlines()
    data = []
    for line in lines:
        if line.startswith('data'):
            vals = line[7:-2].split()
            vals = [float(s) for s in vals]
            data.append(vals)

    data = np.stack(data)
    return data




        