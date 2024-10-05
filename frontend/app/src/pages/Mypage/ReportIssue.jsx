import React, { useState } from 'react';
import '../../styles/ReportIssue.css';
import { useNavigate } from 'react-router-dom';
import GoBackButton from '../../components/GobackButton.jsx';
import Picture from '../../assets/Camera.png';

function ReportIssue() {
  const [selectedIssue, setSelectedIssue] = useState(''); // 선택된 옵션의 값을 저장하는 상태
  const [isEditable, setIsEditable] = useState(false); // 입력 창을 열 수 있는 상태
  const [selectedFile, setSelectedFile] = useState(null); // 선택된 파일을 저장하는 상태
  const navigate = useNavigate();

  const handleSelectChange = (event) => {
    if (event.target.value === "custom") {
      setIsEditable(true); // "신고할 제목을 입력해주세요" 선택 시 입력 가능 상태로 전환
      setSelectedIssue(''); // 커스텀 값을 선택할 경우 빈 값으로 설정
    } else {
      setIsEditable(false); // 다른 값을 선택하면 드롭다운으로 유지
      setSelectedIssue(event.target.value); // 선택된 옵션의 값을 상태에 저장
    }
  };

  const handleFileChange = (event) => {
    const file = event.target.files[0];
    if (file) {
      setSelectedFile(file); // 파일 상태 업데이트
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
        {!isEditable ? ( // 입력 상태가 아닌 경우 드롭다운을 표시
          <select 
            name='issue' 
            className='ReportIssue_select' 
            value={selectedIssue} 
            onChange={handleSelectChange}
          >
            <option>신고할 제목을 입력 해주세요</option>
            <option value="charging">전기차 충전이 안돼요</option>
            <option value="damage">로봇이 파손됐어요</option>
            <option value="not-arrived">로봇이 도착을 안해요</option>
            <option value="not-moving">로봇이 움직이지 않아요</option>
            <option value="custom">직접 입력하기</option>
          </select>
        ) : ( // 입력 상태인 경우 입력 필드로 전환
          <input
            type="text"
            name='customIssue'
            className='ReportIssue_select' // 동일한 클래스 적용
            placeholder='신고할 제목을 입력해주세요'
            value={selectedIssue}
            onChange={(e) => setSelectedIssue(e.target.value)} // 입력된 값 업데이트
          />
        )}

        {/* 파일 입력 필드, 숨겨두고 버튼으로 트리거 */}
        <input
          type="file"
          id="fileInput"
          style={{ display: 'none' }} // 파일 입력 필드를 숨김
          onChange={handleFileChange}
          accept="image/*" // 이미지 파일만 선택 가능
        />

        <button className='putpicture_button' onClick={() => document.getElementById('fileInput').click()}>
          <img src={Picture} alt="사진 삽입" />
        </button>

        {selectedFile && <div>첨부 파일: {selectedFile.name}</div>} {/* 선택한 파일 이름 표시 */}

        <textarea
          className='issue_contents'
          placeholder='신고할 내용을 작성해주세요'
        ></textarea>

        <button className='ReportIssue_startbutton'> 신고하기</button>
      </div>
    </div>
  );
}

export default ReportIssue;
