import React, { useEffect, useState } from 'react';
import '../../styles/ReportIssue.css';
import { useNavigate } from 'react-router-dom';
import GoBackButton from '../../components/GobackButton.jsx';
import Picture from '../../assets/Camera.png';
import axios from 'axios';
import { useRecoilState, useRecoilValue } from 'recoil';
import { reportState } from '../../recoil/ReportAtom.jsx';
import { accessTokenState } from '../../recoil/LoginAtom.jsx';

function ReportIssue() {
  const [selectedIssue, setSelectedIssue] = useState('');
  const [isEditable, setIsEditable] = useState(false);
  const [selectedFile, setSelectedFile] = useState(null);
  const [parkingLots, setParkingLots] = useState([]);
  const [selectedParkingLot, setSelectedParkingLot] = useState('');
  const [chargers, setChargers] = useState([]);
  const [selectedCharger, setSelectedCharger] = useState('');
  const [reportData, setReportData] = useRecoilState(reportState);
  const accessToken = useRecoilValue(accessTokenState);
  const navigate = useNavigate();

  const handleFileChange = (event) => {
    const file = event.target.files[0];
    setSelectedFile(file);
  };

  useEffect(() => {
    const fetchParkingLots = async () => {
      const latitude = 0;
      const longitude = 0;
      try {
        const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/parking-lot', {
          params: {
            latitude,
            longitude,
          },
        });
        if (response.data.code === 200) {
          setParkingLots(response.data.data);
        }
      } catch (error) {
        console.error('Error fetching parking lots:', error);
      }
    };

    fetchParkingLots();
  }, []);

  const fetchChargers = async (parkingLotId) => {
    try {
      const response = await axios.get(`https://j11c208.p.ssafy.io/api/v1/parking-lot/${parkingLotId}/chargers`);
      if (response.data.code === 200) {
        setChargers(response.data.data);
      }
    } catch (error) {
      console.error('Error fetching chargers:', error);
    }
  };

  const handleParkingLotChange = (event) => {
    const parkingLotId = event.target.value;
    setSelectedParkingLot(parkingLotId);
    fetchChargers(parkingLotId);
  };

  const handleSelectChange = (event) => {
    if (event.target.value === "ETC") {
      setIsEditable(true);
      setSelectedIssue('');
    } else {
      setIsEditable(false);
      setSelectedIssue(event.target.value);
    }
  };

  const handleSubmit = async () => {
  const formData = new FormData();

  // 필수 데이터 확인
  if (!selectedCharger) {
    console.error("충전기가 선택되지 않았습니다.");
    return;
  }

  if (!selectedIssue) {
    console.error("문제가 선택되지 않았습니다.");
    return;
  }

  if (!reportData.content) {
    console.error("신고 내용이 없습니다.");
    return;
  }

  // 신고 데이터 준비
  const report = {
    serialNumber: selectedCharger,
    type: selectedIssue,
    content: reportData.content,
    // image 필드는 파일 객체로 추가
  };

  console.log("제출할 데이터:", report); // 제출할 데이터 확인

  // report 키로 추가
  formData.append('report', new Blob([JSON.stringify(report)], { type: 'application/json' }));
  // formData.append('report', JSON.stringify(report)); // JSON으로 변환하여 추가

  // 파일 추가
  if (selectedFile) {
    formData.append('image', selectedFile); // 파일 객체를 추가
  }

  // 콘솔에 FormData 내용 출력
  for (const [key, value] of formData.entries()) {
    console.log(key, value);
  }

  console.log('접속 토큰:', accessToken); // 접속 토큰 확인

  try {
    const response = await axios.post('https://j11c208.p.ssafy.io/api/v1/reports', formData, {
      headers: {
        'Authorization': `Bearer ${accessToken}`,
        'Content-Type': 'multipart/form-data' 
      },
    });

    if (response.data.code === 200) {
      alert('신고가 성공적으로 제출되었습니다.');
      navigate('/');
    } else {
      console.error('응답 오류:', response.data);
    }
  } catch (error) {
    if (error.response) {
      console.error('신고 제출 중 오류:', error.response.data);
      alert(`서버 오류: ${error.response.data.error}`); // 사용자에게 오류 메시지 알리기
    } else {
      console.error('신고 제출 중 오류:', error.message);
      alert(`오류: ${error.message}`); // 사용자에게 오류 메시지 알리기
    }
  }
};

  
  
  
  return (
    <div>
      <GoBackButton />
      <div className='ReportIssue_contents_box'>
        <div className='ReportIssue_contents1'>신고할 내용을</div>
        <div className='ReportIssue_contents2'>작성해 주세요.</div>
      </div>

      <div className='ReportIssue_content'>
        <select 
          name='parkingLot' 
          className='ReportIssue_select' 
          value={selectedParkingLot} 
          onChange={handleParkingLotChange}
        >
          <option value="">주차장을 선택하세요</option>
          {parkingLots.map((lot) => (
            <option key={lot.id} value={lot.id}>{lot.name}</option>
          ))}
        </select>

        <select 
          name='charger' 
          className='ReportIssue_select' 
          value={selectedCharger} 
          onChange={(e) => setSelectedCharger(e.target.value)}
        >
          <option value="">충전기를 선택하세요</option>
          {chargers.map((charger) => (
            <option key={charger.id} value={charger.serialNumber}>{charger.serialNumber}</option>
          ))}
        </select>

        {!isEditable ? (
          <select 
            name='issue' 
            className='ReportIssue_select' 
            value={selectedIssue} 
            onChange={handleSelectChange}
          >
            <option>신고할 제목을 입력 해주세요</option>
            <option value="FLIPPED">로봇이 뒤집혔어요</option>
            <option value="BROKEN">로봇이 고장났어요</option>
            <option value="DAMAGED">로봇이 파손됐어요</option>
            <option value="NOT_CHARGING">로봇이 충전이 안돼요</option>
            <option value="ETC">직접 입력하기</option>
          </select>
        ) : (
          <input
            type="text"
            name='customIssue'
            className='ReportIssue_select'
            placeholder='신고할 제목을 입력해주세요'
            value={selectedIssue}
            onChange={(e) => {
              setSelectedIssue(e.target.value);
              setReportData((prevData) => ({ ...prevData, type: e.target.value }));
            }}
          />
        )}

        <input
          type="file"
          id="fileInput"
          style={{ display: 'none' }}
          onChange={handleFileChange}
          accept="image/*"
        />
        
        <button className='putpicture_button' onClick={() => document.getElementById('fileInput').click()}>
          <img src={Picture} alt="사진 삽입" />
        </button>

        {selectedFile && <div>첨부 파일: {selectedFile.name}</div>}

        <textarea
          className='issue_contents'
          placeholder='신고할 내용을 작성해주세요'
          onChange={(e) => {
            setReportData((prevData) => ({ ...prevData, content: e.target.value }));
          }}
        ></textarea>

        <button className='ReportIssue_startbutton' onClick={handleSubmit}> 신고하기</button>
      </div>
    </div>
  );
}

export default ReportIssue;
