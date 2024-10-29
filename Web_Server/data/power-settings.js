



function saveLoadSettings() {
    // Получаем значения из полей ввода модального окна для настройки нагрузки
    const maxLoad = parseFloat(document.getElementById("maxLoad").value);  // Максимальная нагрузка (кВт)
    const outputVoltage = parseFloat(document.getElementById("outputVoltage").value);  // Выходное напряжение (В)
    const maxCurrentDifference = parseFloat(document.getElementById("maxCurrentDifference").value);  // Максимальная разница тока (А)

    // Проверяем корректность введенных данных
    if (isNaN(maxLoad) || isNaN(outputVoltage) || isNaN(maxCurrentDifference)) {
        alert("Все поля должны быть заполнены корректными значениями.");
        return;  // Выходим из функции, если данные некорректны
    }

    // Формируем строку в формате x-www-form-urlencoded
    const urlEncodedData = `maxLoad=${encodeURIComponent(maxLoad)}&outputVoltage=${encodeURIComponent(outputVoltage)}&maxCurrentDifference=${encodeURIComponent(maxCurrentDifference)}`;

    // Отправляем POST-запрос на сервер для сохранения настроек нагрузки
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
        document.getElementById("loadConfirmationMessage").innerText = "Настройки успешно сохранены!";
        document.getElementById("loadConfirmationMessage").style.display = "block";  // Показываем сообщение
        setTimeout(() => {
            document.getElementById("loadConfirmationMessage").style.display = "none";  // Скрываем сообщение через 3 секунды
        }, 3000);
    })
    .catch(error => {
        console.error('Error:', error);  // Логируем ошибки
    });
}
