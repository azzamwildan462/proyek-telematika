from flask import Flask, jsonify
import mysql.connector
import schedule
import time
from flask_cors import CORS, cross_origin
import threading

app = Flask(__name__)
CORS(app)

# Konfigurasi database
db_config = {
    'user': 'root',
    'password': 'Hertzman01',
    'host': 'localhost',
    'database': 'gps',
    'raise_on_warnings': True
}

latest_data = {}  # Store the latest data globally

def fetch_latest_data():
    try:
        # Membuat koneksi ke database
        conn = mysql.connector.connect(**db_config)
        cursor = conn.cursor()

        # Menjalankan query untuk mengambil data pada row terakhir
        query = "SELECT latitude, longitude FROM gps4 ORDER BY timestamp DESC LIMIT 1"
        cursor.execute(query)

        # Mendapatkan hasil query
        result = cursor.fetchone()

        # Menutup kursor dan koneksi ke database
        cursor.close()
        conn.close()

        if result:
            latitude, longitude = result
            latest_data['latitude'] = latitude
            latest_data['longitude'] = longitude
        else:
            latest_data.clear()  # Clear the latest data if no data is found

    except Exception as e:
        print('Error:', str(e))

def update_latest_data():
    fetch_latest_data()  # Fetch the latest data immediately
    schedule.every(1).minutes.do(fetch_latest_data)  # Update data every minute

# Start updating latest data in the background
update_latest_data()

@app.route('/data')
@cross_origin(origin='http://localhost:3000')
def get_data():
    fetch_latest_data()  # Fetch the latest data before returning the response

    if latest_data:
        data = [{'latitude': latest_data['latitude'], 'longitude': latest_data['longitude']}]
    else:
        data = []  # Empty list if no data is available

    # Mengembalikan hasil dalam format JSON
    return jsonify({'data': data})

def run_flask_app():
    app.run()

if __name__ == '__main__':
    # Start Flask app in a separate thread
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.start()

    while True:
        schedule.run_pending()  # Run pending scheduled tasks
        time.sleep(1)
