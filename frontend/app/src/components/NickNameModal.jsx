import React, { useState } from 'react';
import axios from 'axios';
import { useRecoilValue } from 'recoil';
import { accessTokenState } from '../recoil/LoginAtom';
import '../components/styles/NickNameModal.css'; // 모달 관련 스타일 파일을 따로 생성해 주세요.

function NicknameModal({ isOpen, onClose, currentNickname, onNicknameChange }) {
  const [newNickname, setNewNickname] = useState(currentNickname);
  const accessToken = useRecoilValue(accessTokenState); // Access Token 가져오기

  const handleSubmit = async () => {
    try {
      const response = await axios.patch(
        'https://j11c208.p.ssafy.io/api/v1/users/nickname',
        { nickname: newNickname }, 
        {
          headers: {
            Authorization: `Bearer ${accessToken}`, // Access Token 추가
          },
        }
      );

      if (response.data.code === 200) {
        onNicknameChange(newNickname); // 닉네임 변경
        onClose(); // 모달 닫기
      } else {
        console.error('닉네임 변경 실패:', response.data.message);
      }
    } catch (error) {
      console.error('닉네임 변경 오류:', error);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="Nickmodal">
      <div className="Nickmodal-content">
        <h3>닉네임 변경</h3>
        <input
          type="text"
          value={newNickname}
          onChange={(e) => setNewNickname(e.target.value)}
        />
        <div className="Nickmodal-buttons">
          <button onClick={handleSubmit} className='Nickmodal-change-buttons'>변경</button>
          <button onClick={onClose} className='Nickmodal-cancel-buttons'>취소</button>
        </div>
      </div>
    </div>
  );
}

export default NicknameModal;
