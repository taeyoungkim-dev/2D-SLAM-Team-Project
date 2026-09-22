import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../core/app_config.dart';
import '../state/robot_controller.dart';

/// Local Backend Server(FastAPI)의 IP/포트를 설정하는 화면.
///
/// 데모 당일 로봇/서버 PC의 LAN IP가 매번 바뀔 수 있으므로, 코드를 다시
/// 빌드하지 않고 앱에서 직접 바꿀 수 있게 한다.
class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late final TextEditingController _hostController;
  late final TextEditingController _portController;
  bool _testing = false;
  String? _testResult;

  @override
  void initState() {
    super.initState();
    final config = context.read<AppConfig>();
    _hostController = TextEditingController(text: config.host);
    _portController = TextEditingController(text: config.port.toString());
  }

  @override
  void dispose() {
    _hostController.dispose();
    _portController.dispose();
    super.dispose();
  }

  Future<void> _save() async {
    final config = context.read<AppConfig>();
    final port = int.tryParse(_portController.text.trim()) ?? config.port;
    await config.update(host: _hostController.text.trim(), port: port);
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('서버 설정을 저장했습니다.')),
      );
    }
  }

  Future<void> _testConnection() async {
    setState(() {
      _testing = true;
      _testResult = null;
    });
    final controller = context.read<RobotController>();
    final ok = await controller.api.ping();
    setState(() {
      _testing = false;
      _testResult = ok ? '연결 성공' : '연결 실패';
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('서버 설정')),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const Text(
              'Local Backend Server(FastAPI)가 실행 중인 PC의 로컬 IP와 포트를 '
              '입력하세요. (예: 192.168.0.10 / 8000)',
            ),
            const SizedBox(height: 16),
            TextField(
              controller: _hostController,
              decoration: const InputDecoration(
                labelText: '서버 IP',
                border: OutlineInputBorder(),
              ),
              keyboardType: TextInputType.number,
            ),
            const SizedBox(height: 12),
            TextField(
              controller: _portController,
              decoration: const InputDecoration(
                labelText: '포트',
                border: OutlineInputBorder(),
              ),
              keyboardType: TextInputType.number,
            ),
            const SizedBox(height: 24),
            FilledButton(onPressed: _save, child: const Text('저장')),
            const SizedBox(height: 8),
            OutlinedButton(
              onPressed: _testing ? null : _testConnection,
              child: _testing
                  ? const SizedBox(
                      width: 16,
                      height: 16,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    )
                  : const Text('연결 테스트'),
            ),
            if (_testResult != null)
              Padding(
                padding: const EdgeInsets.only(top: 8),
                child: Text(
                  _testResult!,
                  style: TextStyle(
                    color: _testResult == '연결 성공' ? Colors.green : Colors.red,
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }
}
