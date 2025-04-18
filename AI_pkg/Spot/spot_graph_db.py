import sqlite3
import json

import sqlite3
import json

class SpotGraphDB:
    def __init__(self, db_path="spot_graph.db"):
        self.conn = sqlite3.connect(db_path)
        self._create_tables()

    def _create_tables(self):
        cursor = self.conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS nodes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL, y REAL, z REAL,
                rx REAL, ry REAL, rz REAL,
                joint_angle TEXT,
                is_visited INTEGER,
                up_node_id INTEGER,
                down_node_id INTEGER,
                left_node_id INTEGER,
                right_node_id INTEGER,
                front_node_id INTEGER,
                back_node_id INTEGER,
                rx_plus_node_id INTEGER,
                rx_minus_node_id INTEGER,
                ry_plus_node_id INTEGER,
                ry_minus_node_id INTEGER,
                rz_plus_node_id INTEGER,
                rz_minus_node_id INTEGER,
                FOREIGN KEY (up_node_id) REFERENCES nodes(id),
                FOREIGN KEY (down_node_id) REFERENCES nodes(id),
                FOREIGN KEY (left_node_id) REFERENCES nodes(id),
                FOREIGN KEY (right_node_id) REFERENCES nodes(id)
                FOREIGN KEY (front_node_id) REFERENCES nodes(id),
                FOREIGN KEY (back_node_id) REFERENCES nodes(id),
                FOREIGN KEY (rx_plus_node_id) REFERENCES nodes(id),
                FOREIGN KEY (rx_minus_node_id) REFERENCES nodes(id),
                FOREIGN KEY (ry_plus_node_id) REFERENCES nodes(id),
                FOREIGN KEY (ry_minus_node_id) REFERENCES nodes(id),
                FOREIGN KEY (rz_plus_node_id) REFERENCES nodes(id),
                FOREIGN KEY (rz_minus_node_id) REFERENCES nodes(id)
            )
        ''')
        self.conn.commit()

    def add_node(self, node):
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT id FROM nodes
            WHERE x=? AND y=? AND z=? AND rx=? AND ry=? AND rz=?
        ''', (*node.base_position, *node.base_rotation))
        result = cursor.fetchone()
        if result:
            return result[0]

        cursor.execute('''
            INSERT INTO nodes (
                x, y, z, rx, ry, rz, joint_angle, is_visited,
                up_node_id, down_node_id, left_node_id, right_node_id,
                front_node_id, back_node_id, rx_plus_node_id, rx_minus_node_id,
                ry_plus_node_id, ry_minus_node_id, rz_plus_node_id, rz_minus_node_id
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            *node.base_position, *node.base_rotation,
            json.dumps(node.joint_angle), int(node.is_visited),
            None, None, None, None, None, None,
            None, None, None, None, None, None,
        ))
        self.conn.commit()
        return cursor.lastrowid

    def update_direction_link(self, from_id, to_id, direction):
        cursor = self.conn.cursor()
        if direction not in {"up", "down", "left", "right", "front", "back", "rx_plus", "rx_minus", "ry_plus", "ry_minus", "rz_plus", "rz_minus"}:
            raise ValueError("Direction must be one of: up, down, left, right, front, back, rx_plus, rx_minus, ry_plus, ry_minus, rz_plus, rz_minus")            

        sql = f'UPDATE nodes SET {direction}_node_id = ? WHERE id = ?'
        cursor.execute(sql, (to_id, from_id))
        self.conn.commit()

    def get_direction_neighbors(self, node_id):
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT up_node_id, down_node_id, left_node_id, right_node_id, 
                     front_node_id, back_node_id, rx_plus_node_id, rx_minus_node_id,
                     ry_plus_node_id, ry_minus_node_id, rz_plus_node_id, rz_minus_node_id
            FROM nodes
            WHERE id = ?
        ''', (node_id,))
        result = cursor.fetchone()

        if result:
            return {
                "up": result[0],
                "down": result[1],
                "left": result[2],
                "right": result[3],
                "front": result[4], 
                "back": result[5],
                "rx_plus": result[6],
                "rx_minus": result[7],
                "ry_plus": result[8],
                "ry_minus": result[9],
                "rz_plus": result[10],
                "rz_minus": result[11]
            }
        else:
            return {}

    def get_node_id(self, position, rotation):
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT id FROM nodes
            WHERE x=? AND y=? AND z=? AND rx=? AND ry=? AND rz=?
        ''', (*position, *rotation))
        result = cursor.fetchone()
        return result[0] if result else None

    def get_node_position_by_id(self, node_id):
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT id, x, y, z, rx, ry, rz, joint_angle, is_visited
            FROM nodes
            WHERE id = ?
        ''', (node_id,))
        result = cursor.fetchone()

        if result:
            return {
                "id": result[0],
                "position": [result[1], result[2], result[3]],
                "rotation": [result[4], result[5], result[6]],
                "joint_angle": json.loads(result[7]),
                "is_visited": bool(result[8]),             
            }
        else:
            return None
        
    def close(self):
        self.conn.close()


class SpotNode:
    def __init__(self, base_position, base_rotation):
        self.base_position = base_position
        self.base_rotation = base_rotation
        self.joint_angle = [0] * 12
        self.is_visited = False

# Example usage
if __name__ == "__main__":
    node_a = SpotNode([0, 0, 0], [0, 0, 0])
    node_b = SpotNode([2, 0, 0], [0, 0, 0])
    node_c = SpotNode([1, 3, 2], [0, 0, 0])

    db = SpotGraphDB("test.db")

    from_id = db.add_node(node_a)
    to_id = db.add_node(node_b)
    node_c_id = db.add_node(node_c)

    db.update_direction_link(from_id, to_id, "right")
    db.update_direction_link(from_id, node_c_id, "front")  # 不會重複 right

    neighbors = db.get_direction_neighbors(from_id)
    print("Neighbors of node A:", neighbors)

    # db.close()


    add_node_times = []
    add_connection_times = []
    i = 1
    import time
    start_time = time.perf_counter()
    for _ in range(14000):
        node = SpotNode([0, 0, 0], [0, 0, 0])
        node.joint_angle = [999] * 12
        t0 = time.perf_counter()
        db.add_node(node)
        t1 = time.perf_counter()
        add_node_times.append(t1 - t0)


        db.update_direction_link(i, i+1, "right")
        t2 = time.perf_counter()
        add_connection_times.append(t2 - t1)

        db.get_direction_neighbors(i)
        i+=1

    end_time = time.perf_counter()

    print(f"add_node time: {end_time - start_time:.2f} seconds")
    
    import matplotlib.pyplot as plt
    plt.figure(figsize=(10, 5))
    plt.plot(add_node_times, label='add_node time', alpha=0.7)
    plt.plot(add_connection_times, label='add_connection time', alpha=0.7)
    plt.xlabel("Processed Node Index")
    plt.ylabel("Time (seconds)")
    plt.title("Time Cost per add_node and add_connection")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    db.close()