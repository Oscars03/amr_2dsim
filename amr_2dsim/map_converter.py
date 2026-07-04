import cv2
import numpy as np
import json

def image_to_world_json(image_path, json_path, resolution=0.05):
    # resolution: กำหนดให้ 1 พิกเซล = 0.05 เมตร (ค่ามาตรฐานของ ROS 2)
    
    # 1. โหลดภาพเป็น Grayscale (ภาพขาวดำ)
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # 2. Inverted Threshold: แปลงสีดำ (กำแพง) ให้เป็นสีขาว 
    # เพื่อให้ฟังก์ชันของ OpenCV สามารถค้นหา "เส้นตรง" ได้
    _, thresh = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY_INV)
    
    # 3. ค้นหาเส้นตรงด้วยอัลกอริทึม Hough Line Transform
    lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, threshold=50, minLineLength=20, maxLineGap=5)
    
    walls = []
    if lines is not None:
        # หาจุดกึ่งกลางภาพ เพื่อใช้เป็นจุดพิกัดกำเนิด (0,0) ของโลกจำลอง
        height, width = img.shape
        cx, cy = width / 2, height / 2
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 4. แปลงจากพิกเซลเป็นพิกัด "เมตร"
            # หมายเหตุ: แกน Y ในภาพมีทิศทางชี้ลงล่าง แต่ใน ROS 2 ชี้ขึ้นบน เราจึงต้องใส่เครื่องหมายลบ (-) ที่แกน Y
            m_x1 = (x1 - cx) * resolution
            m_y1 = -(y1 - cy) * resolution
            m_x2 = (x2 - cx) * resolution
            m_y2 = -(y2 - cy) * resolution
            
            walls.append([[m_x1, m_y1], [m_x2, m_y2]])
    
    # 5. บันทึกข้อมูลลงไฟล์ JSON
    world_data = {"walls": walls}
    with open(json_path, 'w') as f:
        json.dump(world_data, f, indent=4)
        
    print(f"แปลงรูปภาพสำเร็จ! พบกำแพงทั้งหมด {len(walls)} เส้น")
    print(f"บันทึกไฟล์ไปที่: {json_path}")

# วิธีใช้งาน: เปลี่ยนชื่อไฟล์ภาพของคุณตรงนี้
image_to_world_json('my_map.png', 'world.json')