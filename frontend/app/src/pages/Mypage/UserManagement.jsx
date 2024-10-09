import React, { useEffect, useState } from 'react';
import { useRecoilValue } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import '../../styles/UserManagement.css';
import Goback_Button from '../../components/GobackButton';
import logoImage from '../../assets/WebLogo.png'; // 이미지 경로 추가

function HistoryItem({ data }) {
  return (
    <div className="history_content">
      <div>시작 시간: {data.startTime}</div>
      <div>종료 시간: {data.endTime}</div>
      <div>충전량: {data.chargeAmount} kWh</div>
      <div>충전 비용: {data.chargeCost} 원</div>
    </div>
  );
}

function UserManagement() {
  const accessToken = useRecoilValue(accessTokenState);
  const [historyDataList, setHistoryDataList] = useState([]);

  useEffect(() => {
    // 데이터 fetching 함수 (예시)
    const fetchHistoryData = async () => {
      try {
        // 여기에 API 호출을 추가하세요.
        // 예시로 임시 데이터를 사용합니다.
        const response = [
          {
            startTime: "2024-10-08T13:58:36",
            endTime: "2024-10-08T14:58:36",
            chargeAmount: 50,
            chargeCost: 15000
          },
          {
            startTime: "2024-10-07T10:30:00",
            endTime: "2024-10-07T11:15:00",
            chargeAmount: 30,
            chargeCost: 9000
          },
          {
            startTime: "2024-10-06T08:00:00",
            endTime: "2024-10-06T09:00:00",
            chargeAmount: 40,
            chargeCost: 12000
          }
        ];
        setHistoryDataList(response);
      } catch (error) {
        console.error('데이터 불러오기 오류:', error);
      }
    };

    fetchHistoryData();
  }, [accessToken]);

  return (
    <div className="UserManagement_container">
      <Goback_Button />
      <div className="UserPatch_box">
        <img src={logoImage} alt="Web Logo" className="logoImage" /> {/* 로고 이미지 추가 */}
        <h3 className="Hi_contents">강효린님, 안녕하세요!</h3>
        <button className="nextButton">{'>'}</button> {/* '>' 버튼 추가 */}
      </div>

      <br />
      <div>이용내역 조회</div>
      {/* 이력 조회를 위한 박스 */}
      <div className="history_box">
        {historyDataList.map((data, index) => (
          <HistoryItem key={index} data={data} />
        ))}
      </div>
    </div>
  );
}

export default UserManagement;
