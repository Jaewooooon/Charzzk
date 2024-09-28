import React from 'react';
import '../../styles/CarManagement.css';
import CarSample from '../../assets/car_sample.png';
import { CircularProgressbar, buildStyles } from 'react-circular-progressbar';
import 'react-circular-progressbar/dist/styles.css';
import { useSpring, animated } from '@react-spring/web'; // 애니메이션을 위한 import 추가
import Slider from 'react-slick';
import GoBackButton from '../../components/GobackButton';


// CircularProgressbar를 애니메이션 가능한 컴포넌트로 만듦
const AnimatedCircularProgressbar = animated(CircularProgressbar);



function CarManagement() {
  const progressValue = 60;

  // 애니메이션 설정: 0에서 지정된 progressValue까지 증가
  const props = useSpring({
    from: { value: 0 },
    to: { value: progressValue },
    config: { duration: 500 }, // 애니메이션 지속 시간 (밀리초 단위)
  });

  var settings = {
    dots: true,
    infinite: true,
    speed: 500,
    slidesToShow: 1,
    slidesToScroll: 1,
  };


  return (
    <div className='CarManagement_ContainerBox'>
      <GoBackButton />
      <div className='Car_ImageBox'>
        <img src={CarSample} alt="Car Sample" className="Car_Image_Management" />
      </div>

      <div className='Circular_ProgressBox'>
        {/* AnimatedCircularProgressbar 컴포넌트를 사용하여 애니메이션 적용 */}
        <AnimatedCircularProgressbar
          value={props.value} // 애니메이션 값으로 설정
          text={props.value.to(val => `${Math.round(val)}%`)} // 텍스트도 애니메이션 값에 맞춰 표시
          className="circular-progressbar"
          styles={buildStyles({
            textColor: 'rgba(214, 214, 214, 0.7)',
            pathColor: '#65F9D1',
            trailColor: 'rgba(214, 214, 214, 0.5)',
            textSize: '15px',
            strokeLinecap: 'butt',  // 끝 모양 설정
          })}
          strokeWidth={15} // 두께 조절
        />
      </div>
    </div>
  );
}

export default CarManagement;
