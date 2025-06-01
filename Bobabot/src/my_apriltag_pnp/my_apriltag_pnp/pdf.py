from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader

img_path = '/home/huiyu/turtlebot_ws/src/BubbleBot/src/my_apriltag_pnp/my_apriltag_pnp/mycode.jpg'
output_pdf = 'qr_a4.pdf'

w, h = A4
margin_x, margin_y = 30, 30

qr_size = 6 * 28.35  # 6 cm = 170.1 points
cols, rows = 3, 4    # Maximum: 3 columns × 4 rows per page
gap = 20             # Approx. 0.7 cm gap

c = canvas.Canvas(output_pdf, pagesize=A4)

for idx in range(cols * rows):
    col = idx % cols
    row = idx // cols
    x = margin_x + col * (qr_size + gap)
    y = h - margin_y - (row + 1) * (qr_size + gap)
    c.drawImage(ImageReader(img_path), x, y, qr_size, qr_size)

c.save()
print("QR PDF generated: qr_a4.pdf (each QR is 6cm), suitable for A4 printing.")
