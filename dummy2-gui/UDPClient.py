import sqlite3
import time
import threading

# Function to create an in-memory SQLite database
def create_database():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS data (
                      id INTEGER PRIMARY KEY,
                      value TEXT)''')
    conn.commit()
    return conn

# Function to continuously insert data into the database at approximately 10ms intervals
def insert_data():
    conn = create_database()
    cursor = conn.cursor()
    count = 0
    while True:
        value = f"Data_{count}"
        cursor.execute("INSERT INTO data (value) VALUES (?)", (value,))
        conn.commit()
        print(f"Inserted: {value}")
        time.sleep(0.01)  # 10ms delay
        count += 1
    conn.close()

# Function to continuously remove data from the database at approximately 10ms intervals
def remove_data():
    conn = create_database()
    cursor = conn.cursor()
    while True:
        cursor.execute("DELETE FROM data WHERE id IN (SELECT id FROM data ORDER BY id ASC LIMIT 1)")
        conn.commit()
        print("Deleted oldest data")
        time.sleep(0.01)  # 10ms delay
    conn.close()

# Main function to start insert and remove operations in separate threads
def main():
    # Start insert and remove operations in separate threads
    insert_thread = threading.Thread(target=insert_data)
    remove_thread = threading.Thread(target=remove_data)
    
    insert_thread.start()
    remove_thread.start()
    
    insert_thread.join()
    remove_thread.join()

if __name__ == "__main__":
    main()
