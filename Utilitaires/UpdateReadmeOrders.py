#!/usr/bin/python3
# -*- coding: utf-8 -*-

from re import findall
from re import MULTILINE
from os import path

projectPath = path.dirname(path.abspath(__file__))+"/.."
projectPath = path.abspath(projectPath)

with open(projectPath+"/src/COM/Order/Orders.h", 'r') as orderFile:
    # Find all order definitions and extract their descriptions and arguments
    orders = findall(r'(?:.*@Description: (.*)\n)?'
                     r'(?:.*@Arguments?: (.*)\n)?'
                     r'(?:.*\*/\n)?'
                     r'(?:.*ORDER\((.*),.*\))', orderFile.read(), MULTILINE)

# Add table headers
orders.insert(0, ("Description", "Arguments", "Ordre"))

# Each string is a row of the table
table = [""] * (len(orders) + 1)  # Add one row for the table header separator

columnOrder = [2, 0, 1]  # Change the order to match readme order
# Build the table column per column
for i in columnOrder:
    results = [order[i] for order in orders]

    maxLen = len(max(results, key=lambda cell_text: len(cell_text))) + 2  # Max entry length per column

    results.insert(1, ":" + "-" * (maxLen - 2) + ":")  # Add table header separator

    for index in range(len(results)):
        result = results[index]
        cell = ["|"]
        cell += [" "] * int((maxLen - len(result)) / 2)
        cell += [result]
        cell += [" "] * int((maxLen - len(result)) / 2)

        # Adds padding to the right of cells that don't have the same parity as the longest one
        if maxLen % 2 != len(result) % 2:
            cell += [" "]

        if i == columnOrder[-1]:  # Close the row
            cell += ["|\n"]

        table[index] += "".join(cell)

with open(projectPath+"/README.md", 'r+') as readme:
    line = readme.readline()
    while line:
        if line.find("## TABLE DES ORDRES") >= 0:
            # Deletes the rest of the file as it will be replaced
            readme.truncate(readme.tell())
            readme.seek(0, 2)  # Move to the end of the stream properly
            readme.write("\n> Text starting from here is generated automatically. "
                         "**ANY MODIFICATION WILL BE OVERWRITTEN**.\n\n")
            readme.writelines(table)
            break
        line = readme.readline()
