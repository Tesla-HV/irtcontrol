<h1>Передатчик сигналов ИК ПДУ</h1>
<i>Руководство пользователя</i><br /><br />
Устройство служит для автоматизации управления оборудованием путём имитации сигналов ИК пульта дистанционного управления


<h2>Питание</h2>
Постоянный ток<br />
чёрный+синий: общий<br />
красный+жёлтый: +7..+12 вольт<br />
потребляемый ток: 50 мА


<h2>Последовательный порт</h2>
<table border="1">
	<tr>
		<td>Baud Rate</td>
		<td>19200</td>
	</tr>
	<tr>
		<td>Data Bits</td>
		<td>8</td>
	</tr>
	<tr>
		<td>Parity</td>
		<td>None</td>
	</tr>
	<tr>
		<td>Stop Bits</td>
		<td>1</td>
	</tr>
	<tr>
		<td>Flow Control</td>
		<td>None</td>
	</tr>
</table>

<h2>Команды управления</h2>
<table border="1">
	<tr>
		<th>Команда</th>
		<th>Пример использования</th>
		<th>Описание</th>
	</tr>
	<tr>
		<td>Txx..xx</td>
		<td>TAB54FE01</td>
		<td>
			Передача последовательности данных.<br />
			В качестве аргумента ожидается целое количество октетов в шестнадцатиричной записи, округлённое в большую сторону. Например, при длине последовательности данных 9 битов ожидается два октета: T01FF.<br />
			Октеты записываются начиная со старших и заканчивая младшими, биты передаются начиная с младших и заканчивая старшими. Непереданные биты старшего октета игнорируются.
			Например, при длине последовательности данных в 9 битов	и команде передачи последовательности T0123 по ИК каналу будет передано: 110001001
		</td>
	</tr>
	<tr>
		<td>R</td>
		<td>R</td>
		<td>Передача маркера повтора</td>
	</tr>
	<tr>
		<td>Dxxxx</td>
		<td>D13C0</td>
		<td>Задержка на указанное количество полупериодов несущей частоты (0000-FFFF)</td>
	</tr>
	<tr>
		<td>Sxx=yyyy</td>
		<td>S07=06AE</td>
		<td>
			Запись в регистр конфигурации
			<ul>
				<li>xx - адрес регистра</li>
				<li>yyyy - значение регистра (0000-FFFF)</li>
			</ul>
		</td>
	</tr>
	<tr>
		<td>L</td>
		<td>L</td>
		<td>
			Чтение всех регистров конфигурации
		</td>
	</tr>
	<tr>
		<td>V</td>
		<td>V</td>
		<td>
			Вывод версии прошивки
		</td>
	</tr>
</table>

<h2>Статус выполнения команды</h2>
<table border="1">
	<tr>
		<th>Статус</th>
		<th>Описание</th>
	</tr>
	<tr>
		<td>OK</td>
		<td>Команда выполнена успешно</td>
	</tr>
	<tr>
		<td>ERROR 01</td>
		<td>Команда не существует</td>
	</tr>
	<tr>
		<td>ERROR 02</td>
		<td>Неверные аргументы команды</td>
	</tr>
	<tr>
		<td>ERROR 03</td>
		<td>Неверный адрес регистра</td>
	</tr>
</table>

<h2>Регистры конфигурации</h2>
Последовательность данных<br />
<img src="https://raw.githubusercontent.com/Tesla-HV/irtcontrol/master/doc/signal.gif" /><br /><br />
Маркер повтора<br />
<img src="https://raw.githubusercontent.com/Tesla-HV/irtcontrol/master/doc/signal_repeat.gif" /><br /><br />

Длительности интервалов задаются количеством полупериодов несущей частоты. Например, для частоты 38 кГц длительность интервала, заданного значением 002A составит примерно 553 мкс<br />
<br />
<table border="1">
	<tr>
		<th>Адрес</th>
		<th>Значение<br />по умолчанию</th>
		<th>Описание</th>
	</tr>
	<tr>
		<td>00</td>
		<td>00D2</td>
		<td>Период несущей частоты (в единицах 1/8000000 сек), по умолчанию рассчитан для частоты 38 кГц</td>
	</tr>
	<tr>
		<td>01</td>
		<td>0020</td>
		<td>Количество значащих битов в последовательности данных</td>
	</tr>
	<tr>
		<td>02</td>
		<td>02AC</td>
		<td>Длительность стартового импульса</td>
	</tr>
	<tr>
		<td>03</td>
		<td>0156</td>
		<td>Длительность паузы после стартового импульса</td>
	</tr>
	<tr class="reserved">
		<td>04</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr class="reserved">
		<td>05</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr>
		<td>06</td>
		<td>002A</td>
		<td>Длительность завершающего импульса</td>
	</tr>
	<tr>
		<td>07</td>
		<td>06AE</td>
		<td>Длительность паузы после завершающего импульса (минимальная)</td>
	</tr>
	<tr class="reserved">
		<td>08</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr class="reserved">
		<td>09</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr>
		<td>0A</td>
		<td>002A</td>
		<td>Длительность импульса "0"</td>
	</tr>
	<tr>
		<td>0B</td>
		<td>002A</td>
		<td>Длительность паузы после импульса "0"</td>
	</tr>
	<tr>
		<td>0C</td>
		<td>002A</td>
		<td>Длительность импульса "1"</td>
	</tr>
	<tr>
		<td>0D</td>
		<td>007E</td>
		<td>Длительность паузы после импульса "1"</td>
	</tr>
	<tr>
		<td>0E</td>
		<td>02AC</td>
		<td>Длительность стартового импульса маркера повтора</td>
	</tr>
	<tr>
		<td>0F</td>
		<td>00AB</td>
		<td>Длительность паузы после стартового импульса маркера повтора</td>
	</tr>
	<tr class="reserved">
		<td>10</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr class="reserved">
		<td>11</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr>
		<td>12</td>
		<td>002A</td>
		<td>Длительность завершающего импульса маркера повтора</td>
	</tr>
	<tr>
		<td>13</td>
		<td>0ED8</td>
		<td>Длительность паузы после завершающего импульса маркера повтора (минимальная)</td>
	</tr>
	<tr class="reserved">
		<td>14</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
	<tr class="reserved">
		<td>15</td>
		<td>0000</td>
		<td>зарезервировано</td>
	</tr>
</table>

<h2>Программное обеспечение</h2>
Использование: irtcontrol ПОРТ КОМАНДА1 [*количество повторов] КОМАНДА2 ...<br />
<br />
Например:<br />
./irtcontrol /dev/ttyUSB0 S02=0160 S03=0158 S06=002A S07=0ED8 S0A=002A S0B=002B S0C=002A S0D=0081 *5 TFB040707 D1000 *5 TF50A0707<br />
<br />
Код возврата:<br />
0 при успешном выполнении каждой команды последовательности, иначе выполнение последовательности прерывается, и возвращается один из следующих кодов:<br /><br />

<table border="1">
	<tr>
		<th>Код возврата</th>
		<th>Сообщение в stderr</th>
		<th>Описание</th>
	</tr>
	<tr>
		<td>255</td>
		<td>Device read timeout</td>
		<td>Нет связи с устройством</td>
	</tr>
	<tr>
		<td>254</td>
		<td>Read buffer overflow (no CR LF)</td>
		<td></td>
	</tr>
	<tr>
		<td>253</td>
		<td>Unable to open serial port</td>
		<td></td>
	</tr>
	<tr>
		<td>252</td>
		<td></td>
		<td>Неверное количество аргументов</td>
	</tr>
	<tr>
		<td>1..3</td>
		<td></td>
		<td>См. "Статус выполнения команды"</td>
	</tr>
</table>