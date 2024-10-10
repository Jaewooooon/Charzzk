import React, { useEffect, useState } from 'react';
import { useRecoilValue } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import '../../styles/UserManagement.css';
import Goback_Button from '../../components/GobackButton';
import axios from 'axios';
import NicknameModal from '../../components/NickNameModal'; // 모달창 컴포넌트 불러오기

function HistoryItem({ data }) {
  const date = data.startTime.split('T')[0];

  return (
    <div className="history_content">
      <div className='History_Name'>{date}</div>
      <div className='History_Box'>
        <div className='History_Title'>차량 종류</div>
        <div className='History_Title2'>{data.car.carType.name}</div>
      </div>
      <div className='History_Box'>
        <div className='History_Title'>차 번호 </div>
        <div className='History_Title2'>{data.car.number}</div>
      </div>
      <div className='History_Box'>
        <div  className='History_Title'>운행 시간 </div>
        <div className='History_Title2'>{data.startTime.split('T')[1]} - {data.endTime.split('T')[1]} </div>
      </div>
      <div className='History_Box'>
        <div className='History_Title'> 충전량 </div>
        <div className='History_Title2'>{Math.floor(data.chargeAmount)} kWh</div>
      </div>
      <div className='History_Box'>
        <div className='History_Title'>충전 비용 </div>
        <div className='History_Title2'> {data.chargeCost} 원</div>
      </div>
    </div>
  );
}

function UserManagement() {
  const accessToken = useRecoilValue(accessTokenState);
  const [historyDataList, setHistoryDataList] = useState([]);
  const [nickname, setNickname] = useState(''); // 닉네임 상태
  const [username, setUsername] = useState(''); // 이메일 상태
  const [isModalOpen, setIsModalOpen] = useState(false); // 모달창 상태

  // 이용 내역 API 호출
  useEffect(() => {
    const fetchHistoryData = async () => {
      try {
        const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/charging-log', {
          headers: {
            Authorization: `Bearer ${accessToken}`, // Access Token을 Authorization 헤더에 추가
          },
        });

        if (response.data.code === 200) {
          // 데이터를 상태에 저장, 최신순으로 정렬
          const sortedData = response.data.data.sort((a, b) => new Date(b.startTime) - new Date(a.startTime));
          setHistoryDataList(sortedData);
        } else {
          console.error('API 응답 오류:', response.data.message);
        }
      } catch (error) {
        console.error('데이터 불러오기 오류:', error);
      }
    };

    fetchHistoryData();
  }, [accessToken]);

  // 사용자 정보 API 호출
  useEffect(() => {
    const fetchUserData = async () => {
      try {
        const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/users/me', {
          headers: {
            Authorization: `Bearer ${accessToken}`, // Access Token을 Authorization 헤더에 추가
          },
        });

        if (response.data.code === 200) {
          // 닉네임과 이메일 상태에 저장
          setNickname(response.data.data.nickname);
          setUsername(response.data.data.username);
        } else {
          console.error('사용자 정보 API 응답 오류:', response.data.message);
        }
      } catch (error) {
        console.error('사용자 정보 불러오기 오류:', error);
      }
    };

    fetchUserData();
  }, [accessToken]);

  // 닉네임 모달창 열기
  const openModal = () => setIsModalOpen(true);

  // 닉네임 모달창 닫기
  const closeModal = () => setIsModalOpen(false);


  return (
    <div className="UserManagement_container">
      <Goback_Button />
      <div className="UserPatch_box" onClick={openModal}> {/* 닉네임 변경 모달 열기 */}
        <div>
          <h3 className="Hi_contents">{nickname}님, 안녕하세요!</h3> {/* 닉네임 출력 */}
          <div className='UserEmail'>{username}</div> 
        </div>
      </div>
      
      <br />
      <div>이용내역 조회</div>
      <div className="history_box">
        {historyDataList.map((data) => (
          <HistoryItem key={data.id} data={data} /> 
        ))}
      </div>

      {/* 닉네임 변경 모달 */}
      <NicknameModal
        isOpen={isModalOpen}
        onClose={closeModal}
        currentNickname={nickname}
        onNicknameChange={setNickname}
      />
    </div>
  );
}

export default UserManagement;
