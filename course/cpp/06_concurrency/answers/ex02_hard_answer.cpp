#include <cassert>
#include <iostream>
#include <mutex>

struct Account {
    int balance;
    std::mutex mutex;
};

void transfer(Account& from, Account& to, int amount) {
    std::scoped_lock lock{from.mutex, to.mutex};
    from.balance -= amount;
    to.balance += amount;
}

int main() {
    Account a{100};
    Account b{50};
    transfer(a, b, 25);
    assert(a.balance == 75);
    assert(b.balance == 75);
    std::cout << "ex02_hard_answer passed\n";
}

