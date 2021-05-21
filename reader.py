import csv

if __name__ == "__main__":
    with open('phi_list.csv') as f:
        reader = csv.reader(f)
        for row in reader:
            print(row)