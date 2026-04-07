import math

# ECEF -> 위경도 변환 함수 (WGS84)
def ecef_to_lla(x, y, z):
    a = 6378137.0
    f = 1 / 298.257223563
    b = a * (1 - f)
    e2 = (a**2 - b**2) / a**2
    ep2 = (a**2 - b**2) / b**2
    p = math.sqrt(x**2 + y**2)
    theta = math.atan2(z * a, p * b)
    lat = math.atan2(z + ep2 * b * math.sin(theta)**3, p - e2 * a * math.cos(theta)**3)
    lon = math.atan2(y, x)
    n = a / math.sqrt(1 - e2 * math.sin(lat)**2)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt

# Point Cloud 스타일이 적용된 KML 생성 함수
def create_kml_from_txt(input_txt, output_kml):
    with open(input_txt, 'r') as f_in, open(output_kml, 'w') as f_out:
        # KML 시작
        f_out.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f_out.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f_out.write('<Document>\n')
        f_out.write('  <name>Point Cloud Data</name>\n')
        
        # 🌟 핵심: Point Cloud용 스타일 정의
        f_out.write('  <Style id="pointCloudStyle">\n')
        f_out.write('    <IconStyle>\n')
        f_out.write('      <color>ff00ffff</color>\n') # 색상 (AABBGGRR 포맷, 예: 노란색)
        f_out.write('      <scale>0.15</scale>\n') # 아이콘 크기를 0.15배로 대폭 축소
        f_out.write('      <Icon>\n')
        f_out.write('        <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n') # 작은 원형 아이콘
        f_out.write('      </Icon>\n')
        f_out.write('    </IconStyle>\n')
        f_out.write('  </Style>\n')
        
        # 데이터 변환 및 기록
        for line in f_in:
            parts = line.strip().split()
            if len(parts) >= 3:
                x, y, z = map(float, parts[:3])
                lat, lon, alt = ecef_to_lla(x, y, z)
                
                # 각 점에 생성한 스타일(#pointCloudStyle) 적용
                f_out.write('  <Placemark>\n')
                f_out.write('    <styleUrl>#pointCloudStyle</styleUrl>\n')
                f_out.write('    <Point>\n')
                f_out.write('      <altitudeMode>absolute</altitudeMode>\n')
                f_out.write(f'      <coordinates>{lon},{lat},{alt}</coordinates>\n')
                f_out.write('    </Point>\n')
                f_out.write('  </Placemark>\n')
                
        # KML 종료
        f_out.write('</Document>\n')
        f_out.write('</kml>\n')

# 파일 실행 (파일 경로를 실제 환경에 맞게 수정하세요)
create_kml_from_txt('scans_3_ecef_0.5.txt', 'point_cloud_output.kml')