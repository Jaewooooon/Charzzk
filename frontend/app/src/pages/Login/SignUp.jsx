import React, { useEffect } from "react";
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
    const [carTypes, setCarTypes] = useRecoilState(carTypesState); // 차량 기종 상태
    const [selectedCarType, setSelectedCarType] = useRecoilState(selectedCarTypeState); // 선택한 차량 기종
    const [nickname, setNickname] = useRecoilState(nicknameState); // 닉네임 상태
    const [nicknameCheckMessage, setNicknameCheckMessage] = useRecoilState(nicknameCheckMessageState); // 닉네임 중복 확인 메시지 상태
    const [isNicknameAvailable, setIsNicknameAvailable] = useRecoilState(isNicknameAvailableState); // 닉네임 사용 가능 여부
    const [carNumber, setCarNumber] = useRecoilState(carNumberState); // 차량 번호
    const [carNickname, setCarNickname] = useRecoilState(carNicknameState); // 차량 별명
    const accessToken = useRecoilValue(accessTokenState); // accessToken을 컴포넌트 최상단에서 가져옵니다.

    // 차량 기종 목록 가져오기
    const fetchCarTypes = async () => {
        try {
            const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/car-types');
            console.log('Fetched car types:', response.data.data); // 응답 데이터 로깅

            if (response.data.code === 200) {
                setCarTypes(response.data.data); // 차량 기종 설정
            } else {
                console.error('Error fetching car types:', response.data.message);
                setCarTypes([]); // 에러 발생 시 빈 배열 설정
            }
        } catch (error) {
            console.error('Error fetching car types:', error);
            setCarTypes([]); // 에러 발생 시 빈 배열 설정
        }
    };

    // 닉네임 중복 확인 함수
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
                setNicknameCheckMessage(response.data.data); // 응답 메시지 저장
                setIsNicknameAvailable(true); // 닉네임 사용 가능
            } else {
                setNicknameCheckMessage('닉네임 사용이 불가합니다.');
                setIsNicknameAvailable(false); // 닉네임 사용 불가능
            }
        } catch (error) {
            console.error('Error checking nickname:', error);
            setNicknameCheckMessage('오류가 발생했습니다.');
            setIsNicknameAvailable(false); // 닉네임 사용 불가능
        }
    };

    // 회원가입 완료 POST 요청 함수
    const handleSignUp = async () => {
        try {

            if (!nickname) {
                alert('닉네임을 입력해 주세요.');
                return;
            }

            const nicknameUpdateResponse = await axios.patch(
                'https://j11c208.p.ssafy.io/api/v1/users/nickname',
                { nickname }, // 입력한 nickname을 사용
                {
                    headers: {
                        Authorization: `Bearer ${accessToken}`, // 헤더에 accessToken 포함
                    }
                }
            );
    
            if (nicknameUpdateResponse.data.code !== 200) {
                console.error('닉네임 변경 실패:', nicknameUpdateResponse.data.message);
                return; // 닉네임 변경 실패 시 더 이상 진행하지 않음
            }
    
            console.log('닉네임 변경 완료:', nicknameUpdateResponse.data);

            
            const requestData = {
                carTypeId: Number(selectedCarType), // selectedCarType은 carTypeId로 전달되어야 합니다.
                number: carNumber,                   // 입력한 차량 번호
                nickname: carNickname                // 입력한 차량 별명
            };
    
            console.log('Request Data:', requestData); // 요청 데이터를 출력해 확인
            console.log('Access Token:', accessToken);
            
            const response = await axios.post(
                'https://j11c208.p.ssafy.io/api/v1/cars',
                requestData,
                {
                    headers: {
                        Authorization: `Bearer ${accessToken}`, // 헤더에 accessToken을 포함
                    }
                }
            );

            if (response.data.code === 200) {
                console.log('회원가입 완료:', response.data);
                navigate('/');
            } else {
                console.error('회원가입 실패:', response.data.message);
            }
        } catch (error) {
            console.error('Error during sign up:', error);
            if (error.response) {
                console.error('Response data:', error.response.data); // 서버 응답 내용 출력
            }
        }
    };

    useEffect(() => {
        fetchCarTypes(); // 컴포넌트 마운트 시 차량 기종 가져오기
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

            {/* 닉네임 중복 확인 메시지 출력 */}
            {nicknameCheckMessage && (
                <div style={{ color: isNicknameAvailable ? 'green' : 'red' }}>
                    {nicknameCheckMessage}
                </div>
            )}

            <div>
                <div>차량 기종</div>
                <select
                    value={selectedCarType}
                    onChange={(e) => setSelectedCarType(e.target.value)} // 선택한 기종을 상태로 저장
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
                    value={carNumber}
                    onChange={(e) => setCarNumber(e.target.value)} 
                />

                <div>차량 별명</div>
                <input 
                    type="text" 
                    value={carNickname}
                    onChange={(e) => setCarNickname(e.target.value)} 
                /><br />
                
                <button onClick={handleSignUp}>회원가입 완료</button>
            </div>
        </>
    );
};

export default SignUp;
