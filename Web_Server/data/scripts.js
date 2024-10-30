
function saveSettings() {
    const settings = {
        max_current: document.getElementById('maxCurrentRange').value,
        voltage_limit: document.getElementById('voltageLimit').value,
        speed_limit: document.getElementById('speedLimit').value
    };
    
    fetch('/save_settings', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(settings),
    }).then(response => {
        if (response.ok) {
            console.log('Настройки сохранены успешно');
        } else {
            console.error('Ошибка сохранения настроек');
        }
    });
}


function saveBatterySettings() {
    // Получаем значения из полей ввода модального окна
    const lowThreshold = parseFloat(document.getElementById("lowThreshold").value);
    const highThreshold = parseFloat(document.getElementById("highThreshold").value);
    const maxCurrentRange = document.getElementById("maxCurrentRange").value;
    // Проверяем, что нижний порог меньше верхнего на 1 В
    if (lowThreshold >= highThreshold - 1) {
        alert("Нижний порог должен быть как минимум на 1 В ниже верхнего порога.");
        return;  // Выходим из функции, не отправляя данные
    }

    // Формируем строку в формате x-www-form-urlencoded
    const urlEncodedData = `lowThreshold=${encodeURIComponent(lowThreshold)}&highThreshold=${encodeURIComponent(highThreshold)}&maxCurrentRange=${encodeURIComponent(maxCurrentRange)}`;

    // Отправляем POST-запрос на сервер
    fetch('/post', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded'
        },
        body: urlEncodedData  // Передаем данные в формате x-www-form-urlencoded
    })
    .then(response => response.text())  // Ожидаем ответ в виде текста
   .then(result => {
    console.log('Success:', result);  // Логируем успешный результат
       // Отображаем сообщение о подтверждении
      document.getElementById("confirmationMessage").innerText = "Настройки успешно сохранены!";
      document.getElementById("confirmationMessage").style.display = "block"; // Показываем сообщение
      setTimeout(() => {
        document.getElementById("confirmationMessage").style.display = "none"; // Скрываем сообщение
        }, 3000);

    })
    .catch(error => {
        console.error('Error:', error);  // Логируем ошибки
    });
}


// Устанавливаем уровень заряда (от 0 до 100)
let batteryLevel = 100;

// Обновляем ширину и цвет заливки в зависимости от уровня заряда
function updateBatteryFill(level) {
    const batteryFill = document.getElementById('batteryFill');
    
    // Рассчитываем сдвиг по оси X относительно уровня заряда
    const maxFillWidth = 3300; // максимальная ширина батареи
    const fillWidth = (level / 100) * maxFillWidth;
    const xPosition = 320 + (maxFillWidth - fillWidth); // сдвиг вправо по мере разряда

    // Меняем x-координату и ширину заливки
    batteryFill.setAttribute('x', xPosition);
    batteryFill.setAttribute('width', fillWidth);

    // Меняем цвет заливки в зависимости от уровня заряда
    if (level > 50) {
        // От зеленого к желтому
        batteryFill.setAttribute('fill', `rgb(${255 - (level - 50) * 5.1}, 255, 0)`);
    } else {
        // От желтого к красному
        batteryFill.setAttribute('fill', `rgb(255, ${level * 5.1}, 0)`);
    }
 }

// Пример динамического изменения уровня заряда
let currentLevel = 100;
setInterval(() => {
    currentLevel -= 1;
    if (currentLevel < 0) {
        currentLevel = 100;
    }
    updateBatteryFill(currentLevel);
}, 100);


            document.addEventListener("DOMContentLoaded", function() {
            // Получаем элементы модальных окон
                const batteryModal = document.getElementById('batteryModal');
                const inverterModal = document.getElementById('inverterModal');
                const loadModal = document.getElementById('loadSettingsModal');
                const WindTurbineModal= document.getElementById(`WindTurbineModal`);
                const FlyWheelModal = document.getElementById('FlyWheelModal');
                // Получаем иконки
                const batteryIcon = document.getElementById('batteryIcon');
                const inverterIcon = document.getElementById('inverterIcon');
                const loadIcon = document.getElementById('loadIcon'); 
                const WindTurbineIcon= document.getElementById(`WindTurbineIcon`);
                const FlyWheelIcon = document.getElementById('FlyWheelIcon');
                // Получаем кнопки закрытия модальных окон 
                const closeBattery = document.getElementById('closeBattery');
                const closeInverter = document.getElementById('closeInverter');
                const closeLoad = document.getElementById('closeLoadSettings');
                const closeWindTurbine = document.getElementById(`closeWindTurbine`);
                const closeFlyWheel = document.getElementById('closeFlyWheel');
                // Открытие модального окна для батареи
                batteryIcon.onclick = function() {
                    batteryModal.style.display = 'flex';
                }
                
                // Открытие модального окна для инвертора
                inverterIcon.onclick = function() {
                    inverterModal.style.display = 'flex';
                    
                }
                  // Открытие модального окна для нагрузки
                loadIcon.onclick = function() {
                    loadModal.style.display = 'flex';
                }
                  // Открытие модального окна для ветрогенератора
                  WindTurbineIcon.onclick = function() {
                    WindTurbineModal.style.display = 'flex';
                }
               
                 // Открытие модального окна для FlyWheel
                 FlyWheelIcon.onclick = function() {
                    FlyWheelModal.style.display = 'flex';
                }
                // Закрытие модальных окон
                closeBattery.onclick = function() {
                    batteryModal.style.display = 'none';
                }
                closeInverter.onclick = function() {
                    inverterModal.style.display = 'none';
                }
                closeLoad.onclick = function() {
                    loadModal.style.display = 'none'; 
                }
                closeWindTurbine.onclick = function() {
                    WindTurbineModal.style.display = 'none'; 
                }

                closeFlyWheel.onclick = function() {
                    FlyWheelModal.style.display = 'none';
                }
                // Закрытие модального окна при клике вне его

                window.onclick = function(event) {
                    if (event.target == batteryModal) {
                        batteryModal.style.display = 'none';
                    }
                    if (event.target == inverterModal) {
                        inverterModal.style.display = 'none';
                    }
                    if (event.target == loadModal) { 
                        loadModal.style.display = 'none';
                    }
                    if (event.target == WindTurbineModal) { 
                        WindTurbineModal.style.display = 'none';
                    }
                    if (event.target == FlyWheelModal) { 
                        FlyWheelModal.style.display = 'none';
                    }
                }
            });
    

            function updateNetworkInfo() {
                fetch('/get_ip')
                    .then(response => {
                        console.log('Ответ от сервера:', response);
                        return response.json();
                    })
                    .then(data => {
                        console.log('Полученные данные:', data);
                        // Обновляем базовые данные (IP, шлюз, маска)
                        document.getElementById('ip_address').textContent = data.ip;
                        document.getElementById('netmask').textContent = data.netmask;
                        document.getElementById('gateway').textContent = data.gateway;
                        document.getElementById('network_mode').textContent = data.mode;
                        document.getElementById('wifi_ssid_display').textContent = data.ssid;  
            
            
                        // Обработка в зависимости от режима
                        if (data.mode === 'AP') {
                           // document.getElementById('wifi_ssid').value = data.ssid;  // Установка SSID для режима AP
                            document.getElementById('wifi_password').value = data.password;  // Установка пароля
                            document.getElementById('switchToAPButton').disabled = true;
                            document.getElementById('switchToSTAButton').disabled = false;
                        } else if (data.mode === 'STA') {
                           // document.getElementById('wifi_ssid').value = data.ssid;  // Установка SSID точки доступа
                            document.getElementById('wifi_password').value = '';  // Пароль для STA не выводим
                            document.getElementById('switchToAPButton').disabled = false;
                            document.getElementById('switchToSTAButton').disabled = true;
                        }
                    })
                    .catch(error => console.error('Error fetching network info:', error));
            }

            function updateData() {
                const indicator = document.getElementById('connectionIndicator');
                var xhr = new XMLHttpRequest();
                var requestTimedOut = false;
            
                xhr.open("GET", "/data", true);
                
                // Устанавливаем таймаут на 2 секунды (2000 мс)
                xhr.timeout = 2000;
            
                xhr.onreadystatechange = function () {
                    if (xhr.readyState == 4) {
                        if (!requestTimedOut) {
                            if (xhr.status == 200) {
                                // Сервер ответил успешно, обновляем данные
                                var data = JSON.parse(xhr.responseText);
                               
                                console.log(data);
            
                                  // Обновляем данные по напряжению на выходе по трём фазам (v_ac_out)
                                const v_ac_out_1 = data.v_ac_out[0];  // Напряжение фазы A
                                const v_ac_out_2 = data.v_ac_out[1];  // Напряжение фазы B
                                const v_ac_out_3 = data.v_ac_out[2];  // Напряжение фазы C
            
            
                                  // Обновляем данные по напряжению на выходе по трём фазам (v_ac_out)
                                document.getElementById("v_ac_out_1").innerText = data.v_ac_out[0];
                                document.getElementById("v_ac_out_2").innerText = data.v_ac_out[1];
                                document.getElementById("v_ac_out_3").innerText = data.v_ac_out[2];
            
                                // Обновляем данные по току на выходе по трём фазам (I_AC_Out)
                                const I_AC_Out_1 = data.I_AC_Out[0];  // Ток фазы A
                                const I_AC_Out_2 = data.I_AC_Out[1];  // Ток фазы B
                                const I_AC_Out_3 = data.I_AC_Out[2];  // Ток фазы C
            
                                // Обновляем данные по току на выходе по трём фазам (I_AC_Out)
                                document.getElementById("I_AC_Out_1").innerText = data.I_AC_Out[0];
                                document.getElementById("I_AC_Out_2").innerText = data.I_AC_Out[1];
                                document.getElementById("I_AC_Out_3").innerText = data.I_AC_Out[2];
            
                                // Рассчитываем мощность для каждой фазы
                                const powerPhaseA = (v_ac_out_1 * I_AC_Out_1 / 1000).toFixed(2);  // Преобразуем в кВт
                                const powerPhaseB = (v_ac_out_2 * I_AC_Out_2 / 1000).toFixed(2);
                                const powerPhaseC = (v_ac_out_3 * I_AC_Out_3 / 1000).toFixed(2);
            
                                // Обновляем элементы таблицы для мощности
                                document.getElementById("powerPhaseA").innerText = powerPhaseA;
                                document.getElementById("powerPhaseB").innerText = powerPhaseB;
                                document.getElementById("powerPhaseC").innerText = powerPhaseC;
                                // Обновляем другие параметры
                                document.getElementById("total_load").innerText = data.total_load;
                                document.getElementById("temperature").innerText = data.temperature;
                                document.getElementById("batt_voltage").innerText = data.batt_voltage;
                                document.getElementById("batt_curr").innerText = data.batt_curr;
                                document.getElementById("speed").innerText = data.speed;
                                document.getElementById("Signal").innerText = data.Signal;
               
            
                                // Обновляем данные графика
                                updateChart(data.speed);
            
                                // Соединение установлено, индикатор становится зелёным
                                indicator.classList.add('active');
                            } else {
                                // Сервер ответил с ошибкой, соединение считается потерянным
                                indicator.classList.remove('active');
                            }
                        }
                    }
                };
                
                // Обработка ошибок сети
                xhr.onerror = function () {
                    // Ошибка соединения, индикатор становится красным
                    indicator.classList.remove('active');
                };
            
                // Обработка таймаута
                xhr.ontimeout = function () {
                    // Запрос занял слишком много времени, считаем, что соединение потеряно
                    requestTimedOut = true;
                    indicator.classList.remove('active');
                };
                
                xhr.send();
            }           