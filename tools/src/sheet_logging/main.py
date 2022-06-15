import zipfile
from black import main
from bs4 import BeautifulSoup
import numpy as np
import json
import re
import os
import pandas as pd

DATA = "./data"


class DataExtractor:
    def __init__(self):
        self.time = []
        self.score = []
        self.x = []
        self.y = []
        self.r = []
        self.angleDiff = []

        self.extract_data()

    def scrape(self, path: str):
        with open(path) as f:
            soup = BeautifulSoup(f, "html.parser")

        time = soup.find("span", class_="record-time").text
        time = time[: time.find("/")]  # remove icon from text

        score = soup.find(
            "div", class_="dashboard-score"
        ).div.span.next_sibling.next_sibling.text
        score = score.split(" ")[0]

        self.time.append(time)
        self.score.append(score)

    def read_zip(self, path: str):
        files = zipfile.ZipFile(path, "r")

        # extract target2 laser accuracy
        result = json.loads(files.read("result.json").decode("utf-8"))
        snapshot = result["Target2"][0]

        # create 2d numpy array and average it
        arr = [
            (snapshot[str(i)]["x"], snapshot[str(i)]["y"], snapshot[str(i)]["r"])
            for i in range(10)
        ]
        arr = np.array(arr)
        # x, y, r is the mean value
        self.x.append(sum(arr[:, 0]) / 10)
        self.y.append(sum(arr[:, 1]) / 10)
        self.r.append(sum(arr[:, 2]) / 10)

        # find last angleDiff
        log = files.read("adb.log").decode("utf-8")
        regex = re.compile("angleDiff = \d+.\d+")
        l = regex.findall(log)

        self.angleDiff.append(l[len(l) - 1].split(" ")[2])

    def extract_data(self):
        for i in range(3):
            path = f"{DATA}/{i+1}"
            for file in os.listdir(path):
                _, suffix = os.path.splitext(file)

                if suffix == ".html":
                    self.scrape(f"{path}/{file}")
                elif suffix == ".zip":
                    self.read_zip(f"{path}/{file}")


def clear_data():
    for i in range(3):
        path = f"{DATA}/{i+1}"
        for file in os.listdir(path):
            f_path = f"{path}/{file}"

            if os.path.isfile(f_path):
                os.remove(f_path)


def update_csv(path: str):
    # creat default data.csv
    if not os.path.exists(path):
        f = open(path, "w")
        f.write("No.,Score,Accuracy,X_distance,Y_distance,Time,Type")
        f.close()

    df = pd.read_csv(path)
    
    # create new dataframe and concat it
    if df.last_valid_index():
        number = df.last_valid_index() + 1
    else:
        number = 1

    dxt = DataExtractor()
    arr = []
    for i in range(len(dxt.x)):
        arr.append(
            {
                "No.": number + i,
                "Score": dxt.score[i],
                "Accuracy": dxt.r[i],
                "X_distance": dxt.x[i],
                "Y_distance": dxt.y[i],
                "Time": dxt.time[i],
                "Type": "Random",
            }
        )

    new_df = pd.DataFrame(data=arr)
    df = pd.concat([df, new_df], ignore_index=True)
    pd.DataFrame.to_csv(df, path, index=False)


def main():
    # creating data folder
    if not os.path.exists("./data"):
        os.mkdir("./data")
        
    for i in range(3):
        if not os.path.exists(f"./data/{i+1}"):
            os.mkdir(f"./data/{i+1}")

    while True:
        command = input("Please type command (u, c, e): ")
        if command == "exit":
            break
        if command == "clear":
            clear_data()
        if command == "update":
            update_csv("./data.csv")

    return


if __name__ == "__main__":
    main()
