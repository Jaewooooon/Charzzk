import React, { useState } from 'react';
import Modal from 'react-modal';
import '../components/styles/AddModal.css';

function AddCarModal({ isOpen, onRequestClose, onConfirm, carTypes }) {
  const [selectedCarType, setSelectedCarType] = useState('');
  const [number, setNumber] = useState('');
  const [nickname, setNickname] = useState('');

  const handleConfirm = () => {
    const newCarData = {
      carTypeId: selectedCarType,
      number,
      nickname,
    };
    onConfirm(newCarData);
  };

  return (
    <Modal isOpen={isOpen} onRequestClose={onRequestClose}>
      <h2>차량 추가</h2>
      <label>
        차량 종류
        <select value={selectedCarType} onChange={e => setSelectedCarType(e.target.value)}>
          <option value="">선택하세요</option>
          {carTypes.map(carType => (
            <option key={carType.id} value={carType.id}>
              {carType.name}
            </option>
          ))}
        </select>
      </label>

      <label>
        차량 번호
        <input type="text" value={number} onChange={e => setNumber(e.target.value)} />
      </label>

      <label>
        차량 닉네임
        <input type="text" value={nickname} onChange={e => setNickname(e.target.value)} />
      </label>

      <button className="add-button" onClick={handleConfirm}>추가</button>
      <button className="cancel-button" onClick={onRequestClose}>취소</button>
    </Modal>
  );
}

export default AddCarModal;
