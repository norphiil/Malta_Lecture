import csv
from collections import Counter
import matplotlib.pyplot as plt


def read_data(filename):
    """
    Reads the CSV file and returns a list of dictionaries.
    """
    data = []
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            ingredients = row['Ingredients'].split(",")
            data.append({
                "Recipe_Name": row["Recipe_Name"],
                "Ingredients": ingredients,
                "Dish_Name": row["Dish_Name"]
            })
    return data


def get_all_ingredient_counts(data) -> Counter:
    """
    Calculates the count of each ingredient across all recipes.
    """
    all_ingredients = []
    for recipe in data:
        all_ingredients.extend(recipe["Ingredients"])
    return Counter(all_ingredients)


def get_dish_ingredient_counts(data, dish_name):
    """
    Calculates the count of each ingredient for a specific dish.
    """
    dish_ingredients = []
    for recipe in data:
        if recipe["Dish_Name"] == dish_name:
            dish_ingredients.extend(recipe["Ingredients"])
    return Counter(dish_ingredients)


def create_bar_chart(ingredient_counts, title):
    ingredients, counts = zip(*ingredient_counts)

    plt.figure(figsize=(10, 6))  # Adjust figure size as desired
    plt.bar(ingredients, counts)
    plt.xlabel("Ingredients")
    plt.ylabel("Count")
    plt.title(f"Ingredient Counts for {title}")
    plt.xticks(rotation=45, ha='right')  # Rotate x-axis labels for better readability
    plt.tight_layout()
    plt.show()


# Read the data from the CSV file
data: list = read_data("clean_data.csv")

# Get all ingredient counts
all_ingredient_counts: Counter = get_all_ingredient_counts(data)

# Create a bar chart for all ingredients
create_bar_chart(all_ingredient_counts.most_common(), "All Dishes")

# User input for dish selection
selected_dish = input("Enter a dish name (or 'all' for all dishes): ")

# Get ingredient counts for the selected dish
if selected_dish == "all":
    dish_ingredient_counts = all_ingredient_counts
else:
    dish_ingredient_counts = get_dish_ingredient_counts(data, selected_dish)

# Create a bar chart for the selected dish (if applicable)
if selected_dish != "all":
    create_bar_chart(dish_ingredient_counts.most_common(10), selected_dish)

print("Done!")
