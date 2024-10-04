import React, { useEffect, useState } from 'react';
import '../../styles/CarManagement.css';
import CarSample from '../../assets/car_sample.png';
import { CircularProgressbar, buildStyles } from 'react-circular-progressbar';
import 'react-circular-progressbar/dist/styles.css';
import { useSpring, animated } from '@react-spring/web';
import GoBackButton from '../../components/GobackButton';
import axios from 'axios';
import { useRecoilValue } from 'recoil'; 
import { accessTokenState } from '../../recoil/LoginAtom.jsx';

const AnimatedCircularProgressbar = animated(CircularProgressbar);

function CarManagement() {
  const [carData, setCarData] = useState([]); // 초기값을 빈 배열로 설정
  const accessToken = useRecoilValue(accessTokenState); 
  const progressValue = 60;

  const props = useSpring({
    from: { value: 0 },
    to: { value: progressValue },
    config: { duration: 500 },
  });

  const fetchCarData = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
        headers: {
          Authorization: `Bearer ${accessToken}`,
        },
      });
      
      if (response.data.code === 200) {
        setCarData(response.data.data); 
        console.log('Data:', response.data.data);
      } else {
        console.error('Error fetching car data:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching data:', error);
    }
  };

  useEffect(() => {
    fetchCarData();
  }, []);

  return (
    <div className='CarManagement_ContainerBox'>
      <GoBackButton />
      
      {carData.length > 0 ? (
        <>
          <div className='Car_ImageBox'>
            <img src={carData[0].carType.image} alt="Car Image" className="Car_Image_Management" />
          </div>

          <div className='Circular_ProgressBox'>
            <AnimatedCircularProgressbar
              value={props.value} 
              text={props.value.to(val => `${Math.round(val)}%`)} 
              className="circular-progressbar"
              styles={buildStyles({
                textColor: 'rgba(214, 214, 214, 0.7)',
                pathColor: '#65F9D1',
                trailColor: 'rgba(214, 214, 214, 0.5)',
                textSize: '15px',
                strokeLinecap: 'butt', 
              })}
              strokeWidth={15} 
            />
          </div>

          <div className='Car_Info'>
            <h2>닉네임: {carData[0].nickname}</h2>
            <h3>차량 이름: {carData[0].carType.name}</h3>
            <img src={carData[1].carType.image} alt={carData[0].carType.name} className="Car_Image_Management" />
          </div>
        </>
      ) : (
        <p>차량 정보를 로드 중입니다...</p>
      )}
    </div>
  );
}

export default CarManagement;
