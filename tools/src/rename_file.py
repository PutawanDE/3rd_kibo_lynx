import os


def main():
    path = "./DebugImages"
    files = os.listdir(path)

    for file in files:
        base, suffix = os.path.splitext(file)
        if suffix == "":
            os.rename(f"{path}/{file}", f"{path}/{base}.png")

    return


if __name__ == "__main__":
    main()
