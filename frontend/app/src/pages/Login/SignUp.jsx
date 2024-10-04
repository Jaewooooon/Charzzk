import React, { useEffect, useState } from "react";
import { useRecoilState, useRecoilValue } from 'recoil';
import axios from 'axios';
import GobackButton from '../../components/GobackButton.jsx';
import '../../styles/SignUp.css';
import { 
    carTypesState, 
    selectedCarTypeState, 
    nicknameState, 
    nicknameCheckMessageState, 
    isNicknameAvailableState, 
    carNumberState, 
    carNicknameState 
} from '../../recoil/SignupData.jsx';
import { accessTokenState } from '../../recoil/LoginAtom.jsx';
import { useNavigate } from 'react-router-dom';

const SignUp = () => {
    const navigate = useNavigate();
    const [carTypes, setCarTypes] = useRecoilState(carTypesState);
    const [nickname, setNickname] = useRecoilState(nicknameState);
    const [nicknameCheckMessage, setNicknameCheckMessage] = useRecoilState(nicknameCheckMessageState);
    const [isNicknameAvailable, setIsNicknameAvailable] = useRecoilState(isNicknameAvailableState);
    const accessToken = useRecoilValue(accessTokenState);

    // 차량 정보들을 관리하는 상태
    const [carList, setCarList] = useState([{ carType: '', carNumber: '', carNickname: '' }]);

    // 차량 기종 목록 가져오기
    const fetchCarTypes = async () => {
        try {
            const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/car-types');
            console.log('Fetched car types:', response.data.data);
            if (response.data.code === 200) {
                setCarTypes(response.data.data);
            } else {
                console.error('Error fetching car types:', response.data.message);
                setCarTypes([]);
            }
        } catch (error) {
            console.error('Error fetching car types:', error);
            setCarTypes([]);
        }
    };

    // 닉네임 중복 확인
    const checkNickname = async () => {
        if (!nickname) {
            setNicknameCheckMessage('닉네임을 입력해 주세요.');
            setIsNicknameAvailable(false);
            return;
        }

        try {
            const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/users/check-nickname', {
                params: { nickname }
            });
            console.log('Nickname check response:', response.data);

            if (response.data.code === 200) {
                setNicknameCheckMessage(response.data.data);
                setIsNicknameAvailable(true);
            } else {
                setNicknameCheckMessage('닉네임 사용이 불가합니다.');
                setIsNicknameAvailable(false);
            }
        } catch (error) {
            console.error('Error checking nickname:', error);
            setNicknameCheckMessage('닉네임 사용이 불가합니다.');
            setIsNicknameAvailable(false);
        }
    };

    // 회원가입 완료
    const handleSignUp = async () => {
        try {
            if (!nickname) {
                alert('닉네임을 입력해 주세요.');
                return;
            }

            const nicknameUpdateResponse = await axios.patch(
                'https://j11c208.p.ssafy.io/api/v1/users/nickname',
                { nickname },
                {
                    headers: {
                        Authorization: `Bearer ${accessToken}`,
                    }
                }
            );

            if (nicknameUpdateResponse.data.code !== 200) {
                console.error('닉네임 변경 실패:', nicknameUpdateResponse.data.message);
                return;
            }

            console.log('닉네임 변경 완료:', nicknameUpdateResponse.data);

            const carRequests = carList.map((car) => ({
                carTypeId: Number(car.carType),
                number: car.carNumber,
                nickname: car.carNickname
            }));

            console.log('Request Data:', carRequests);

            for (const requestData of carRequests) {
                const response = await axios.post(
                    'https://j11c208.p.ssafy.io/api/v1/cars',
                    requestData,
                    {
                        headers: {
                            Authorization: `Bearer ${accessToken}`,
                        }
                    }
                );
                if (response.data.code !== 200) {
                    console.error('회원가입 실패:', response.data.message);
                    return;
                }
            }

            console.log('회원가입 완료');
            navigate('/');
        } catch (error) {
            console.error('Error during sign up:', error);
            if (error.response) {
                console.error('Response data:', error.response.data);
            }
        }
    };

    // 차량 추가 버튼 클릭 시 차량 정보 추가
    const addCar = () => {
        setCarList([...carList, { carType: '', carNumber: '', carNickname: '' }]);
    };

    // 차량 정보 입력 핸들러
    const handleCarChange = (index, field, value) => {
        const newCarList = [...carList];
        newCarList[index][field] = value;
        setCarList(newCarList);
    };

    useEffect(() => {
        fetchCarTypes();
    }, []);

    return (
        <>
            <GobackButton />
            <div className='SignUp_contents_box'>
                <div className='SignUp_contents1'>회원정보 및 차량정보를</div>
                <div className='SignUp_contents2'>입력해 주세요.</div>
            </div>

            {/* 닉네임 입력 및 중복 확인 */}
            <div>닉네임</div>
            <input 
                type="text" 
                value={nickname} 
                onChange={(e) => setNickname(e.target.value)} 
            />
            <button onClick={checkNickname}>중복확인</button>
            <br />

            {/* 닉네임 중복 확인 메시지 출력 */}
            {nicknameCheckMessage && (
                <div style={{ color: isNicknameAvailable ? 'green' : 'red' }}>
                    {nicknameCheckMessage}
                </div>
            )}

            {/* 차량 추가 */}
            {carList.map((car, index) => (
                <div key={index}>
                    <div>차량 {index + 1}</div>
                    <div>차량 기종</div>
                    <select
                        value={car.carType}
                        onChange={(e) => handleCarChange(index, 'carType', e.target.value)}
                    >
                        <option value="">차량 기종을 선택하세요</option>
                        {carTypes.length > 0 ? (
                            carTypes.map((carType) => (
                                <option key={carType.id} value={carType.id}>
                                    {carType.name}
                                </option>
                            ))
                        ) : (
                            <option disabled>차량 기종을 불러오는 중입니다...</option>
                        )}
                    </select>

                    <div>차량 번호</div>
                    <input 
                        type="text" 
                        value={car.carNumber}
                        onChange={(e) => handleCarChange(index, 'carNumber', e.target.value)} 
                    />

                    <div>차량 별명</div>
                    <input 
                        type="text" 
                        value={car.carNickname}
                        onChange={(e) => handleCarChange(index, 'carNickname', e.target.value)} 
                    />
                    <br />
                </div>
            ))}

            <button onClick={addCar}>차량 추가</button>
            <br />
            <button onClick={handleSignUp}>회원가입 완료</button>
        </>
    );
};

export default SignUp;
