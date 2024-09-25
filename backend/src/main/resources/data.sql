-- // Insert car_type data
INSERT INTO car_type (name, image)
VALUES
       ('현대 포터2 일렉트릭', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/hyundai_porter2.png'),
       ('현대 아이오닉5', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/hyundai_ioniq5.png'),
       ('현대 아이오닉6', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/hyundai_ioniq6.png'),
       ('현대 넥쏘', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/hyundai_nexo.png'),
       ('현대 코나 일렉트릭', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/hyundai_kona_electric.png'),
       ('기아 EV6', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/kia_ev6.png'),
       ('기아 EV9', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/kia_ev9.png'),
       ('기아 봉고3 EV', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/kia_bongo3_ev.png'),
       ('기아 니로 EV', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/kia_niro_ev.png'),
       ('기아 레이 EV', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/kia_ray_ev.png'),
       ('테슬라 모델Y', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/tesla_model_y.png'),
       ('테슬라 모델3', 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/cars/tesla_model_3.png');

-- // Insert parking-lot data
INSERT INTO parking_lot (parking_lot_id, name, latitude, longitude, image)
VALUES
    (1, '하남산단근로자 공영주차장', 35.1939509, 126.8071044, 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/parking-lot/parking-lot1.png'),
    (2, '수완지구 공영주차장', 35.2003196, 126.8246306, 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/parking-lot/parking-lot2.png'),
    (3, '롯데쇼핑(주)롯데마트수완점주차장', 35.1902505, 126.8209779, 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/parking-lot/parking-lot3.png'),
    (4, '하남3지구 주차타워1', 35.1830841, 126.8034723, 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/parking-lot/parking-lot4.png'),
    (5, '중앙주차장', 35.1824086, 126.8312939, 'https://charzzk-image.s3.ap-northeast-2.amazonaws.com/parking-lot/parking-lot5.png');

-- // Insert parking-spot data
INSERT INTO parking_spot (parking_lot_id, latitude, longitude, name)
VALUES
    (1, 35.1939509, 126.8071044, 'A1'),
    (1, 35.1939509, 126.8071044, 'A2'),
    (1, 35.1939509, 126.8071044, 'A3'),
    (1, 35.1939509, 126.8071044, 'A4'),
    (1, 35.1939509, 126.8071044, 'A5'),
    (1, 35.1939509, 126.8071044, 'A6'),
    (1, 35.1939509, 126.8071044, 'A7'),
    (1, 35.1939509, 126.8071044, 'A8'),
    (1, 35.1939509, 126.8071044, 'A9'),
    (1, 35.1939509, 126.8071044, 'B1'),
    (1, 35.1939509, 126.8071044, 'B2'),
    (1, 35.1939509, 126.8071044, 'B3'),
    (1, 35.1939509, 126.8071044, 'B4'),
    (1, 35.1939509, 126.8071044, 'B5'),
    (1, 35.1939509, 126.8071044, 'B6'),
    (1, 35.1939509, 126.8071044, 'B7'),
    (1, 35.1939509, 126.8071044, 'B8'),
    (1, 35.1939509, 126.8071044, 'B9'),

    (2, 35.1939509, 126.8071044, 'A1'),
    (2, 35.1939509, 126.8071044, 'A2'),
    (2, 35.1939509, 126.8071044, 'A3'),
    (2, 35.1939509, 126.8071044, 'A4'),
    (2, 35.1939509, 126.8071044, 'A5'),
    (2, 35.1939509, 126.8071044, 'A6'),
    (2, 35.1939509, 126.8071044, 'A7'),
    (2, 35.1939509, 126.8071044, 'A8'),
    (2, 35.1939509, 126.8071044, 'A9'),
    (2, 35.1939509, 126.8071044, 'B1'),
    (2, 35.1939509, 126.8071044, 'B2'),
    (2, 35.1939509, 126.8071044, 'B3'),
    (2, 35.1939509, 126.8071044, 'B4'),
    (2, 35.1939509, 126.8071044, 'B5'),
    (2, 35.1939509, 126.8071044, 'B6'),
    (2, 35.1939509, 126.8071044, 'B7'),
    (2, 35.1939509, 126.8071044, 'B8'),
    (2, 35.1939509, 126.8071044, 'B9'),

    (3, 35.1939509, 126.8071044, 'A1'),
    (3, 35.1939509, 126.8071044, 'A2'),
    (3, 35.1939509, 126.8071044, 'A3'),
    (3, 35.1939509, 126.8071044, 'A4'),
    (3, 35.1939509, 126.8071044, 'A5'),
    (3, 35.1939509, 126.8071044, 'A6'),
    (3, 35.1939509, 126.8071044, 'A7'),
    (3, 35.1939509, 126.8071044, 'A8'),
    (3, 35.1939509, 126.8071044, 'A9'),
    (3, 35.1939509, 126.8071044, 'B1'),
    (3, 35.1939509, 126.8071044, 'B2'),
    (3, 35.1939509, 126.8071044, 'B3'),
    (3, 35.1939509, 126.8071044, 'B4'),
    (3, 35.1939509, 126.8071044, 'B5'),
    (3, 35.1939509, 126.8071044, 'B6'),
    (3, 35.1939509, 126.8071044, 'B7'),
    (3, 35.1939509, 126.8071044, 'B8'),
    (3, 35.1939509, 126.8071044, 'B9'),

    (4, 35.1939509, 126.8071044, 'A1'),
    (4, 35.1939509, 126.8071044, 'A2'),
    (4, 35.1939509, 126.8071044, 'A3'),
    (4, 35.1939509, 126.8071044, 'A4'),
    (4, 35.1939509, 126.8071044, 'A5'),
    (4, 35.1939509, 126.8071044, 'A6'),
    (4, 35.1939509, 126.8071044, 'A7'),
    (4, 35.1939509, 126.8071044, 'A8'),
    (4, 35.1939509, 126.8071044, 'A9'),
    (4, 35.1939509, 126.8071044, 'B1'),
    (4, 35.1939509, 126.8071044, 'B2'),
    (4, 35.1939509, 126.8071044, 'B3'),
    (4, 35.1939509, 126.8071044, 'B4'),
    (4, 35.1939509, 126.8071044, 'B5'),
    (4, 35.1939509, 126.8071044, 'B6'),
    (4, 35.1939509, 126.8071044, 'B7'),
    (4, 35.1939509, 126.8071044, 'B8'),
    (4, 35.1939509, 126.8071044, 'B9'),

    (5, 35.1939509, 126.8071044, 'A1'),
    (5, 35.1939509, 126.8071044, 'A2'),
    (5, 35.1939509, 126.8071044, 'A3'),
    (5, 35.1939509, 126.8071044, 'A4'),
    (5, 35.1939509, 126.8071044, 'A5'),
    (5, 35.1939509, 126.8071044, 'A6'),
    (5, 35.1939509, 126.8071044, 'A7'),
    (5, 35.1939509, 126.8071044, 'A8'),
    (5, 35.1939509, 126.8071044, 'A9'),
    (5, 35.1939509, 126.8071044, 'B1'),
    (5, 35.1939509, 126.8071044, 'B2'),
    (5, 35.1939509, 126.8071044, 'B3'),
    (5, 35.1939509, 126.8071044, 'B4'),
    (5, 35.1939509, 126.8071044, 'B5'),
    (5, 35.1939509, 126.8071044, 'B6'),
    (5, 35.1939509, 126.8071044, 'B7'),
    (5, 35.1939509, 126.8071044, 'B8'),
    (5, 35.1939509, 126.8071044, 'B9');

-- // Insert charger data
INSERT INTO charger (parking_lot_id, serial_number, battery, status, created_at, updated_at)
VALUES

    (1, '12345A', 100, 'WAITING', '2024-09-25 10:00:00', '2024-09-25 10:00:00'),
    (1, '12345B', 90, 'WAITING', '2024-09-25 10:05:00', '2024-09-25 10:05:00'),
    (1, '12345C', 80, 'WAITING', '2024-09-25 10:10:00', '2024-09-25 10:10:00'),
    (1, '12345D', 70, 'WAITING', '2024-09-25 10:15:00', '2024-09-25 10:15:00'),
    (1, '12345E', 60, 'ERROR', '2024-09-25 10:20:00', '2024-09-25 10:20:00'),

    (2, '22345A', 100, 'WAITING', '2024-09-25 10:25:00', '2024-09-25 10:25:00'),
    (2, '22345B', 90, 'WAITING', '2024-09-25 10:30:00', '2024-09-25 10:30:00'),
    (2, '22345C', 80, 'WAITING', '2024-09-25 10:35:00', '2024-09-25 10:35:00'),
    (2, '22345D', 70, 'ERROR', '2024-09-25 10:40:00', '2024-09-25 10:40:00'),
    (2, '22345E', 60, 'ERROR', '2024-09-25 10:45:00', '2024-09-25 10:45:00'),

    (3, '32345A', 100, 'CAR_CHARGING', '2024-09-25 10:50:00', '2024-09-25 10:50:00'),
    (3, '32345B', 90, 'CAR_CHARGING', '2024-09-25 10:55:00', '2024-09-25 10:55:00'),
    (3, '32345C', 80, 'CAR_CHARGING', '2024-09-25 11:00:00', '2024-09-25 11:00:00'),
    (3, '32345D', 70, 'ERROR', '2024-09-25 11:05:00', '2024-09-25 11:05:00'),
    (3, '32345E', 60, 'ERROR', '2024-09-25 11:10:00', '2024-09-25 11:10:00'),

    (4, '42345A', 100, 'WAITING', '2024-09-25 11:15:00', '2024-09-25 11:15:00'),
    (4, '42345B', 90, 'WAITING', '2024-09-25 11:20:00', '2024-09-25 11:20:00'),
    (4, '42345C', 80, 'WAITING', '2024-09-25 11:25:00', '2024-09-25 11:25:00'),
    (4, '42345D', 70, 'ERROR', '2024-09-25 11:30:00', '2024-09-25 11:30:00'),
    (4, '42345E', 60, 'ERROR', '2024-09-25 11:35:00', '2024-09-25 11:35:00'),

    (5, '52345A', 100, 'WAITING', '2024-09-25 11:40:00', '2024-09-25 11:40:00'),
    (5, '52345B', 90, 'WAITING', '2024-09-25 11:45:00', '2024-09-25 11:45:00'),
    (5, '52345C', 80, 'WAITING', '2024-09-25 11:50:00', '2024-09-25 11:50:00'),
    (5, '52345D', 70, 'WAITING', '2024-09-25 11:55:00', '2024-09-25 11:55:00'),
    (5, '52345E', 60, 'WAITING', '2024-09-25 12:00:00', '2024-09-25 12:00:00');