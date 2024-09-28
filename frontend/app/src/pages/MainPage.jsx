import React, { useState } from 'react';
import { Link } from 'react-router-dom';  // Link 컴포넌트 임포트
import '../styles/MainPage.css';
import CarSample from '../assets/car_sample.png';
import MypageCar from '../assets/MypageCar.png';
import MyPagePayment from '../assets/MyPagePayment.png';
import MyPageReport from '../assets/MypageReport.png';
import ChargeStatus from '../assets/ChargeStatus.png';
import ChargeStart from '../assets/ChargeStart.png';


const MainPage = () => {
  const [payment, setPayment] = useState(124252);
  const [chargeamout, setchargeamount] = useState(578);

  return (

    <div className='MainPage_ContainerBox'>
      <div className='Car_Information'>
      <img src={CarSample} alt="Car Sample" className="Car_Image" />
      <div className='Car_MonthInformation'>
        <div>
        <div className ='Car_MonthInformation1'>이번달 충전 요금</div>
        <div className='MonthPayment_Contents'><div>{payment}</div><div className='small_font'>원</div></div>
        </div>

      <div>
      <div className ='Car_MonthInformation2'>이번달 충전량</div>
      <div className='MonthCharge_Contents'><div>{chargeamout}</div><div className='small_font'> kWh</div></div>
      </div>
      </div>

      </div>
    <div className='MainPage_ButtonBox'>

    <div className='ChargePage_Content'>충전페이지</div>
          <div className='ChargePage'>
          <div className="ButtonContainer">
          <Link to="/charge-map">
            <button className='ChargeFind_Button'><img src={ChargeStatus} alt="ChargeStatus" className="ChargeStatus_img" />충전구역 조회</button>
          </Link>
          <Link to="/charge-start">
            <button className='ChargeStart_Button'><div className='ChargeStart_Button_Content'>충전하기</div><img src={ChargeStart} alt="ChargeStart" className="ChargeStart_Image" /></button>
          </Link>
        </div>

        <div>
          <Link to="/charge-status">
            <button className='ChargeStatus_Button'>충전현황 확인</button>
          </Link>
        </div>
        </div>

        
        <div className='MyPage_Content'>마이페이지</div>
        <div className='MyPage'>
          <Link to="/mypage/car-management">
            <button className='Mypage_Car'><img src={MypageCar} alt="MypageCar" className="MypageCar_Image" />차량 관리</button>
          </Link>
        
          <Link to="/mypage/payment-management">
            <button className='Mypage_Payment'><img src={MyPagePayment} alt="MyPagePayment" className="MyPagePayment_Image" />결제수단 관리</button>
          </Link>
        
          <Link to="/mypage/report-issue">
            <button className='Mypage_ReportIssue'><img src={MyPageReport} alt="MyPageReport" className="MyPageReport_Image" />신고하기</button>
          </Link>
        </div>
    </div>
    </div>
  );
}

export default MainPage;
